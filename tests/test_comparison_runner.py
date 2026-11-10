"""Exercise batch orchestration without starting terminals or simulation processes."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ComparisonRunnerTests(unittest.TestCase):
    def run_batch(self, failed_controller=None, missing_input=False):
        with tempfile.TemporaryDirectory(prefix='puma batch ') as tmp:
            root = Path(tmp)
            workspace = root / 'arm_bot'
            workspace.mkdir()
            shutil.copy(ROOT / 'arm_bot/all.sh', workspace / 'all.sh')
            (workspace / 'scripts').mkdir()
            shutil.copy(ROOT / 'arm_bot/scripts/desktop_environment.sh', workspace / 'scripts')
            shutil.copy(ROOT / 'arm_bot/scripts/runtime_environment.sh', workspace / 'scripts')
            (workspace / 'install').mkdir()
            (workspace / 'install/setup.bash').write_text(':\n')
            data = root / 'input with spaces'
            data.mkdir()
            if not missing_input:
                (data / 'path_001_joint_states.csv').touch()
            models = root / 'models'
            models.mkdir()
            for name in ['fyp_jax_delan_50.jax', 'best_GRUResidual.pt', 'feature_scaler.pkl']:
                (models / name).touch()
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            for name in ['colcon', 'ros2', 'sleep', 'python3']:
                path = bin_dir / name
                path.write_text('#!/usr/bin/env bash\nexit 0\n')
                path.chmod(0o755)
            calls = root / 'calls.jsonl'
            terminal = bin_dir / 'gnome-terminal'
            terminal.write_text('''#!/usr/bin/python3
import json, os, pathlib, sys
args = sys.argv[sys.argv.index('--') + 1:]
with open(os.environ['PUMA_TEST_CALLS'], 'a') as log:
    log.write(json.dumps(args) + '\\n')
status = pathlib.Path(args[3])
if status.name != 'gazebo.status':
    rc = 7 if status.name.startswith(os.environ.get('PUMA_TEST_FAIL', '!')) else 0
    status.write_text(str(rc) + '\\n')
''')
            terminal.chmod(0o755)
            env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'],
                       PUMA_TEST_CALLS=str(calls), PUMA_MODEL_DIR=str(models))
            env.pop('PUMA_PYTHON', None)
            if failed_controller:
                env['PUMA_TEST_FAIL'] = failed_controller
            result = subprocess.run(
                ['bash', str(workspace / 'all.sh'), '--csv-dir', str(data), '1'],
                env=env, input='\n', text=True, capture_output=True, timeout=10,
            )
            launches = [json.loads(line) for line in calls.read_text().splitlines()] if calls.exists() else []
            return result, launches

    def test_original_controller_order_and_literal_csv_path(self):
        result, launches = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([Path(args[3]).stem for args in launches],
                         ['gazebo', 'dnn_path_001', 'ctc_path_001', 'pid_path_001'])
        for args in launches[1:]:
            self.assertTrue(args[-1].endswith('input with spaces/path_001_joint_states.csv'))

    def test_failed_controller_stops_later_controllers(self):
        result, launches = self.run_batch(failed_controller='ctc')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('ctc_path_001 failed (exit 7)', result.stderr)
        self.assertEqual(len(launches), 3)

    def test_missing_csv_fails_before_any_launch(self):
        result, launches = self.run_batch(missing_input=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('CSV not found', result.stderr)
        self.assertEqual(launches, [])
