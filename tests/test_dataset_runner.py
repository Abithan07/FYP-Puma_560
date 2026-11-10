"""Legacy dataset runs must use consistent IDs and stop on publisher errors."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DatasetRunnerTests(unittest.TestCase):
    def run_dataset(self, fail=False):
        with tempfile.TemporaryDirectory(prefix='puma dataset ') as tmp:
            workspace = Path(tmp)
            scripts = workspace / 'src/scripts'
            scripts.mkdir(parents=True)
            shutil.copy(ROOT / 'arm_bot/src/scripts/dataset_generator.sh', scripts)
            (scripts / 'Joint_states').mkdir()
            (scripts / 'Joint_states/path_001_joint_states.csv').touch()
            bin_dir = workspace / 'bin'
            bin_dir.mkdir()
            calls = workspace / 'calls'
            python = bin_dir / 'python3'
            python.write_text('''#!/usr/bin/env bash
printf '%s\\n' "$*" >> "$PUMA_TEST_CALLS"
if [[ "$1" == *torque_publisher.py ]]; then
    [[ "$PUMA_TEST_FAIL" != 1 ]] || exit 7
    echo 'File saved: /tmp/path_001_log_1.csv'
fi
''')
            python.chmod(0o755)
            sleep = bin_dir / 'sleep'
            sleep.write_text('#!/usr/bin/env bash\nexit 0\n')
            sleep.chmod(0o755)
            env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'],
                       PUMA_TEST_CALLS=str(calls), PUMA_TEST_FAIL='1' if fail else '0')
            result = subprocess.run(['bash', str(scripts / 'dataset_generator.sh'), '1'],
                                    env=env, text=True, capture_output=True, timeout=10)
            return result, calls.read_text().splitlines()

    def test_single_id_uses_three_digits_and_matches_plot_input(self):
        result, calls = self.run_dataset()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(calls), 2)
        self.assertIn('path_001_joint_states.csv', calls[0])
        self.assertIn('path_001_joint_states.csv', calls[1])

    def test_publisher_failure_does_not_plot_or_report_success(self):
        result, calls = self.run_dataset(fail=True)
        self.assertEqual(result.returncode, 7)
        self.assertEqual(len(calls), 1)
