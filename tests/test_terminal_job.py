"""Check failure reporting and argument handling without launching ROS or Gazebo."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'arm_bot/scripts/terminal_job.sh'


class TerminalJobTests(unittest.TestCase):
    def run_job(self, command, setup=':'):
        with tempfile.TemporaryDirectory(prefix='puma terminal ') as tmp:
            workspace = Path(tmp)
            (workspace / 'install').mkdir()
            (workspace / 'install/setup.bash').write_text(setup + '\n')
            status = workspace / 'job.status'
            pid = workspace / 'job.pid'
            result = subprocess.run(
                ['bash', str(HELPER), str(workspace), str(status), str(pid),
                 str(workspace / 'job.log'), *command],
                capture_output=True, text=True, timeout=10,
            )
            return result, status.read_text().strip(), pid.exists()

    def test_success_keeps_arguments_literal(self):
        result, status, has_pid = self.run_job(['printf', '%s\n', 'a path with spaces', '$(literal)'])
        self.assertEqual(result.returncode, 0)
        self.assertEqual(status, '0')
        self.assertTrue(has_pid)
        self.assertIn('a path with spaces\n$(literal)', result.stdout)

    def test_command_failure_reaches_parent(self):
        result, status, _ = self.run_job(['bash', '-c', 'exit 7'])
        self.assertEqual(result.returncode, 7)
        self.assertEqual(status, '7')

    def test_setup_failure_reaches_parent(self):
        result, status, has_pid = self.run_job(['true'], setup='return 9')
        self.assertEqual(result.returncode, 9)
        self.assertEqual(status, '9')
        self.assertFalse(has_pid)

    def test_missing_command_reaches_parent(self):
        result, status, _ = self.run_job(['puma_nonexistent_command'])
        self.assertNotEqual(result.returncode, 0)
        self.assertNotEqual(status, '0')
