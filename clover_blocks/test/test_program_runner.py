import pathlib
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from program_runner import ProgramRunner


class ProgramRunnerTest(unittest.TestCase):
    def setUp(self):
        self.finished = threading.Event()
        self.events, self.errors = [], []

        def running(value):
            self.events.append(value)
            if not value:
                self.finished.set()

        # Execute real Python in a subprocess, without ROS dependencies.
        command = [sys.executable, '-c', 'import sys; exec(compile(open(sys.argv[1]).read(), sys.argv[1], "exec"))']
        self.runner = ProgramRunner(command, running, self.errors.append, stop_timeout=0.2)

    def tearDown(self):
        self.runner.stop()

    def test_system_exit_releases_lifecycle(self):
        self.assertTrue(self.runner.start('raise SystemExit(7)'))
        self.assertTrue(self.finished.wait(5))
        self.assertIsNone(self.runner.process)
        self.assertTrue(self.errors)
        self.finished.clear()
        self.assertTrue(self.runner.start('pass'))
        self.assertTrue(self.finished.wait(5))
        self.assertEqual(self.events, [True, False, True, False])

    def test_immediate_stop_is_not_reset_by_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = pathlib.Path(directory) / 'effect'
            self.assertTrue(self.runner.start('import time; time.sleep(1); open({!r}, "w").close()'.format(str(marker))))
            self.runner.stop()
            self.assertIsNone(self.runner.process)
            self.assertFalse(marker.exists())
            self.assertEqual(self.events, [True, False])

    def test_stop_kills_busy_loop_and_allows_next_run(self):
        self.assertTrue(self.runner.start('while True: pass'))
        self.assertFalse(self.runner.start('pass'))
        self.runner.stop()
        self.assertIsNone(self.runner.process)
        self.assertTrue(self.runner.start('pass'))
        self.assertTrue(self.finished.wait(5))

    def test_stop_escalates_when_sigterm_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = pathlib.Path(directory) / 'ready'
            self.assertTrue(self.runner.start(
                'import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
                'open({!r}, "w").close(); time.sleep(60)'.format(str(ready))))
            for _ in range(100):
                if ready.exists():
                    break
                threading.Event().wait(0.01)
            self.assertTrue(ready.exists())
            self.runner.stop()
            self.assertIsNone(self.runner.process)

    def test_launch_failure_cleans_up(self):
        self.runner.command = ['/nonexistent/clover-worker']
        with self.assertRaises(FileNotFoundError):
            self.runner.start('pass')
        self.assertIsNone(self.runner.process)

    def test_concurrent_run_and_stop_leave_no_active_program(self):
        entered = threading.Event()
        release = threading.Event()
        original_callback = self.runner.on_running

        def running(value):
            if value:
                entered.set()
                release.wait(3)
            original_callback(value)

        self.runner.on_running = running
        start = threading.Thread(target=lambda: self.runner.start('while True: pass'))
        stop = threading.Thread(target=self.runner.stop)
        start.start()
        self.assertTrue(entered.wait(3))
        stop.start()
        release.set()
        start.join(3)
        stop.join(3)
        self.assertFalse(start.is_alive() or stop.is_alive())
        self.assertIsNone(self.runner.process)
        self.assertEqual(self.events, [True, False])


if __name__ == '__main__':
    unittest.main()
