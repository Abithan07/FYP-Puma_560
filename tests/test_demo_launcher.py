"""The final launcher must build and source the workspace from any directory."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DemoLauncherTests(unittest.TestCase):
    def run_launcher(self, option=None, installed=True):
        with tempfile.TemporaryDirectory(prefix='puma final demo ') as tmp:
            root = Path(tmp)
            workspace = root / 'arm_bot'
            workspace.mkdir()
            shutil.copy(ROOT / 'arm_bot/run_demo.sh', workspace)
            if installed:
                (workspace / 'install').mkdir()
                (workspace / 'install/setup.bash').write_text('export PUMA_TEST_SOURCED=yes\n')
            binary = root / 'bin'
            binary.mkdir()
            calls = root / 'calls'
            for command in ['colcon', 'ros2', 'python3']:
                script = binary / command
                script.write_text('#!/usr/bin/env bash\nprintf "%s|%s|%s|%s\\n" "$(basename "$0")" "$PWD" "${PUMA_TEST_SOURCED:-no}" "$*" >> "$PUMA_TEST_CALLS"\n')
                script.chmod(0o755)
            env = dict(os.environ, PATH=str(binary) + os.pathsep + os.environ['PATH'], PUMA_TEST_CALLS=str(calls))
            for name in ['PUMA_PROJECT_ROOT', 'PUMA_DATA_DIR', 'PUMA_MODEL_DIR', 'PUMA_LOG_DIR']:
                env.pop(name, None)
            result = subprocess.run(['bash', str(workspace / 'run_demo.sh'), *([option] if option else [])],
                                    cwd=root, env=env, capture_output=True, text=True, timeout=10)
            return result, calls.read_text().splitlines() if calls.exists() else []

    def test_build_then_source_then_gui(self):
        result, calls = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([line.split('|')[0] for line in calls], ['colcon', 'python3'])
        self.assertIn('|yes|src/scripts/demo/traj_gui.py', calls[-1])
        self.assertTrue(all(line.split('|')[1].endswith('/arm_bot') for line in calls))

    def test_no_build_uses_existing_install(self):
        result, calls = self.run_launcher('--no-build')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0].startswith('python3|'))

    def test_missing_install_is_reported_before_gui(self):
        result, calls = self.run_launcher('--no-build', installed=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Build arm_bot', result.stderr)
        self.assertEqual(calls, [])
