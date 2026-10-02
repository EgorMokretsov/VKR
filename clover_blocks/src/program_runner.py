"""Lifecycle of a Blocks program. User code never runs in the service process."""

import os
import signal
import subprocess
import tempfile
import threading


class ProgramRunner:
    def __init__(self, command, on_running, on_error, stop_timeout=1.0):
        self.command = command
        self.on_running = on_running
        self.on_error = on_error
        self.stop_timeout = stop_timeout
        self.lock = threading.RLock()
        self.process = None
        self.filename = None
        self.stopping = False

    def start(self, code):
        with self.lock:
            if self.process is not None:
                return False
            filename = None
            process = None
            try:
                with tempfile.NamedTemporaryFile(prefix='clover-blocks-', suffix='.py', delete=False) as source:
                    filename = source.name
                    source.write(code.encode('utf-8'))
                process = subprocess.Popen(self.command + [filename], start_new_session=True)
                self.process, self.filename = process, filename
                self.stopping = False
                self.on_running(True)
                monitor = threading.Thread(target=self._monitor, args=(process, filename))
                monitor.daemon = True
                monitor.start()
                return True
            except BaseException:
                if process is not None:
                    self._signal_group(process, signal.SIGKILL)
                    process.wait()
                if filename is not None:
                    os.unlink(filename)
                self.process = self.filename = None
                self.on_running(False)
                raise

    @staticmethod
    def _signal_group(process, sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def _monitor(self, process, filename):
        code = process.wait()
        self._finish(process, filename, code)

    def _finish(self, process, filename, code):
        with self.lock:
            if self.process is not process:
                return
            # Clean up subprocesses even if user code exited via SystemExit.
            self._signal_group(process, signal.SIGKILL)
            try:
                os.unlink(filename)
            except FileNotFoundError:
                pass
            stopped = self.stopping
            self.process = self.filename = None
            try:
                if code and not stopped:
                    self.on_error('Program exited with code {}'.format(code))
            finally:
                self.on_running(False)

    def stop(self):
        with self.lock:
            process = self.process
            if process is None:
                return
            self.stopping = True
            self._signal_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=self.stop_timeout)
            except subprocess.TimeoutExpired:
                self._signal_group(process, signal.SIGKILL)
                process.wait(timeout=self.stop_timeout)
            # A child may still be alive after the group leader exits.
            self._signal_group(process, signal.SIGKILL)
            self._finish(process, self.filename, process.returncode)
