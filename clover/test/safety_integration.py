#!/usr/bin/env python3
"""Regression checks against real ROS nodes, with a mocked FCU (no drone)."""

import math
import socket
import struct
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State, ManualControl, PositionTarget
from std_msgs.msg import Bool, String, Header
from std_srvs.srv import Trigger
from clover.srv import Navigate, SetVelocity
from clover_blocks.srv import Run


class SafetyIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('safety_integration', anonymous=True)
        for service in ['navigate', 'set_velocity', 'land', 'simple_offboard/release', 'clover_blocks/run']:
            rospy.wait_for_service(service, timeout=15)
        cls.navigate = rospy.ServiceProxy('navigate', Navigate)
        cls.set_velocity = rospy.ServiceProxy('set_velocity', SetVelocity)
        cls.state_pub = rospy.Publisher('mavros/state', State, queue_size=1)
        cls.pose_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=1)
        cls.br = tf2_ros.TransformBroadcaster()
        cls.transforms = True
        cls.done = threading.Event()

        def telemetry():
            while not cls.done.wait(0.01):
                now = rospy.Time.now()
                cls.state_pub.publish(State(header=Header(stamp=now), connected=True, armed=True, mode='OFFBOARD'))
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = now, 'map'
                pose.pose.position.z = 1
                pose.pose.orientation.w = 1
                cls.pose_pub.publish(pose)
                if cls.transforms:
                    transform = TransformStamped()
                    transform.header.stamp, transform.header.frame_id = now, 'map'
                    transform.child_frame_id = 'moving'
                    transform.transform.rotation.w = 1
                    cls.br.sendTransform(transform)

        cls.thread = threading.Thread(target=telemetry, daemon=True)
        cls.thread.start()
        time.sleep(1)

    @classmethod
    def tearDownClass(cls):
        cls.done.set()
        cls.thread.join(timeout=2)

    def test_01_current_position_is_finite(self):
        messages = []
        sub = rospy.Subscriber('mavros/setpoint_position/local', PoseStamped, messages.append)
        try:
            self.assertTrue(self.navigate(x=0, y=0, z=1, speed=0.5, frame_id='map').success)
            time.sleep(0.3)
            self.assertTrue(messages)
            for msg in messages:
                p = msg.pose.position
                self.assertTrue(all(math.isfinite(v) for v in [p.x, p.y, p.z]))
                self.assertEqual((p.x, p.y, p.z), (0, 0, 1))
        finally:
            sub.unregister()

    def test_02_udp_rejects_invalid_packets(self):
        messages = []
        sub = rospy.Subscriber('mavros/manual_control/send', ManualControl, messages.append)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            time.sleep(0.3)
            sock.sendto(struct.pack('<hhhh', -1000, 1000, 0, -1), ('127.0.0.1', 35602))
            time.sleep(0.2)
            self.assertEqual(len(messages), 1)
            self.assertEqual(messages[0].x, -1000)
            for packet in [b'', b'\xff', b'\x00' * 7, b'\x00' * 9, b'\x00' * 100,
                           struct.pack('<hhhh', 1001, 0, 0, 0), struct.pack('<hhhh', 0, 0, -1, 0)]:
                sock.sendto(packet, ('127.0.0.1', 35602))
            time.sleep(0.3)
            self.assertEqual(len(messages), 1)
        finally:
            sock.close()
            sub.unregister()

    def test_03_nested_requests_cannot_release_guard(self):
        result = []

        def first():
            result.append(rospy.ServiceProxy('navigate', Navigate)(x=1, y=1, z=1, frame_id='missing'))

        thread = threading.Thread(target=first)
        thread.start()
        try:
            time.sleep(0.1)
            for call in [lambda: self.navigate(x=0, y=0, z=1, frame_id='map'),
                         lambda: self.navigate(x=0, y=0, z=1, frame_id='map'),
                         lambda: rospy.ServiceProxy('land', Trigger)(),
                         lambda: rospy.ServiceProxy('simple_offboard/release', Trigger)()]:
                self.assertTrue(thread.is_alive())
                response = call()
                self.assertFalse(response.success)
                self.assertEqual(response.message, 'Busy')
        finally:
            thread.join(timeout=3)
        self.assertFalse(thread.is_alive())
        self.assertFalse(result[0].success)
        self.assertTrue(self.navigate(x=0, y=0, z=1, frame_id='map').success)

    def test_04_tf_loss_withholds_velocity(self):
        messages = []
        sub = rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget, messages.append)
        try:
            self.assertTrue(self.set_velocity(vx=1, vy=0, vz=0, yaw=0, frame_id='moving').success)
            time.sleep(0.3)
            self.assertTrue(messages)
            self.__class__.transforms = False
            time.sleep(0.5)
            count = len(messages)
            time.sleep(0.3)
            self.assertEqual(len(messages), count, 'stale velocity was republished')
            self.__class__.transforms = True
            time.sleep(0.3)
            self.assertGreater(len(messages), count)
        finally:
            self.__class__.transforms = True
            sub.unregister()

    def test_05_blocks_stop_and_restart_after_system_exit(self):
        run = rospy.ServiceProxy('clover_blocks/run', Run)
        stop = rospy.ServiceProxy('clover_blocks/stop', Trigger)
        running = []
        output = []
        state_sub = rospy.Subscriber('clover_blocks/running', Bool, lambda msg: running.append(msg.data))
        print_sub = rospy.Subscriber('clover_blocks/print', String, lambda msg: output.append(msg.data))
        try:
            time.sleep(0.2)
            self.assertTrue(run(code='while True: pass').success)
            self.assertTrue(stop().success)
            self.assertTrue(run(code='raise SystemExit(7)').success)
            time.sleep(1.5)
            self.assertTrue(run(code='print("restarted")').success)
            time.sleep(1.5)
            self.assertIn('restarted', output)
            self.assertEqual(running[-1], False)
        finally:
            stop()
            state_sub.unregister()
            print_sub.unregister()


if __name__ == '__main__':
    rostest.rosrun('clover', 'safety_integration', SafetyIntegration)
