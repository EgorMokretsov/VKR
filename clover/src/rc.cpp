/*
 * Clover mobile remote control backend
 * Send ManualControl messages through UDP
 * 'latched_state' topic
 *
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <atomic>
#include <thread>
#include "safety.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/ManualControl.h"
#include "mavros_msgs/Mavlink.h"

class RC
{
public:
	RC():
		nh(),
		nh_priv("~")
	{
		bool use_fake_gcs = nh_priv.param("use_fake_gcs", true);
		initLatchedState();
		socket_thread = std::thread(&RC::socketThread, this);

		if (use_fake_gcs) {
			gcs_thread = std::thread(&RC::fakeGCSThread, this);
		}
	}
	~RC() {
		if (socket_thread.joinable()) socket_thread.join();
		if (gcs_thread.joinable()) gcs_thread.join();
	}

private:
	ros::NodeHandle nh, nh_priv;
	std::thread socket_thread, gcs_thread;
	ros::Subscriber state_sub;
	ros::Publisher state_pub;
	ros::Timer state_timeout_timer;
	std::atomic<double> last_manual_control{0};
	mavros_msgs::StateConstPtr state_msg;

	void handleState(const mavros_msgs::StateConstPtr& state)
	{
		state_timeout_timer.setPeriod(ros::Duration(3), true);
		state_timeout_timer.start();

		if (!state_msg ||
			state->connected != state_msg->connected ||
			state->mode != state_msg->mode ||
			state->armed != state_msg->armed) {
				state_msg = state;
				state_pub.publish(state_msg);
			}
	}

	void stateTimedOut(const ros::TimerEvent&)
	{
		ROS_INFO("State timeout");
		mavros_msgs::State unknown_state;
		state_pub.publish(unknown_state);
		state_msg = nullptr;
	}

	void initLatchedState()
	{
		state_sub = nh.subscribe("mavros/state", 1, &RC::handleState, this);
		state_pub = nh.advertise<mavros_msgs::State>("state_latched", 1, true);
		state_timeout_timer = nh.createTimer(ros::Duration(0), &RC::stateTimedOut, this, true, false);

		// Publish initial state
		mavros_msgs::State unknown_state;
		state_pub.publish(unknown_state);
	}

	void fakeGCSThread()
	{
		// Awful workaround for fixing PX4 not sending STATUSTEXTs
		// if there is no GCS heartbeats.
		// TODO: use timer
		// TODO: remove, when PX4 get this fixed.
		ros::Publisher mavlink_pub = nh.advertise<mavros_msgs::Mavlink>("mavlink/to", 1);

		// HEARTBEAT from GCS message
		mavros_msgs::Mavlink hb;
		hb.framing_status = mavros_msgs::Mavlink::FRAMING_OK;
		hb.magic = mavros_msgs::Mavlink::MAVLINK_V20;
		hb.len = 9;
		hb.incompat_flags = 0;
		hb.compat_flags = 0;
		hb.seq = 0;
		hb.sysid = 255;
		hb.compid = 0;
		hb.checksum = 26460;
		hb.payload64.push_back(342282393542983680);
		hb.payload64.push_back(3);

		ros::Rate rate(1);
		while (ros::ok()) {
			if (ros::Time::now().toSec() - last_manual_control.load() < 8) {
				mavlink_pub.publish(hb);
			}
			rate.sleep();
		}
	}

	int createSocket(int port)
	{
		int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sockfd < 0) {
			ROS_FATAL("socket error: %s", strerror(errno));
			ros::shutdown();
			return -1;
		}

		sockaddr_in sin{};
		sin.sin_family = AF_INET;
		// The legacy UDP protocol has no authentication. Keep it local and use
		// an authenticated tunnel for remote control.
		sin.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		sin.sin_port = htons(port);

		if (bind(sockfd, (sockaddr *)&sin, sizeof(sin)) < 0) {
			ROS_FATAL("socket bind error: %s", strerror(errno));
			close(sockfd);
			ros::shutdown();
			return -1;
		}
		timeval timeout{0, 200000};
		if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
			ROS_FATAL("socket timeout error: %s", strerror(errno));
			close(sockfd);
			ros::shutdown();
			return -1;
		}

		return sockfd;
	}

	void socketThread()
	{
		int port;
		nh_priv.param("port", port, 35602);
		if (port < 1 || port > 65535) {
			ROS_FATAL("Invalid RC UDP port: %d", port);
			ros::shutdown();
			return;
		}
		int sockfd = createSocket(port);
		if (sockfd < 0) return;

		unsigned char buff[9]; // Extra byte detects oversized packets.

		ros::Publisher manual_control_pub = nh.advertise<mavros_msgs::ManualControl>("mavros/manual_control/send", 1);
		mavros_msgs::ManualControl manual_control_msg;

		sockaddr_in client_addr{};

		ROS_INFO("UDP RC initialized on port %d", port);

		while (ros::ok()) {
			// read next UDP packet
			socklen_t client_addr_size = sizeof(client_addr);
			int bsize = recvfrom(sockfd, buff, sizeof(buff), 0, (sockaddr *) &client_addr, &client_addr_size);

			if (bsize < 0) {
				if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
					ROS_ERROR_THROTTLE(30, "recvfrom() error: %s", strerror(errno));
				continue;
			}

			clover_safety::ControlMessage msg;
			if (!clover_safety::decodeControl(buff, static_cast<size_t>(bsize), msg)) {
				ROS_ERROR_THROTTLE(30, "Invalid RC UDP packet (size %d)", bsize);
				continue;
			}

			manual_control_msg.x = msg.x;
			manual_control_msg.y = msg.y;
			manual_control_msg.z = msg.z;
			manual_control_msg.r = msg.r;
			manual_control_pub.publish(manual_control_msg);

			last_manual_control.store(ros::Time::now().toSec());
		}
		close(sockfd);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rc");
	RC rc;
	ros::spin();
}
