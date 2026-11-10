"""Snap editor libraries must not leak into host Qt/GTK applications."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
GUARD = ROOT / 'arm_bot/scripts/desktop_environment.sh'
VARIABLES = ('GTK_PATH', 'GIO_MODULE_DIR', 'LD_LIBRARY_PATH', 'QT_PLUGIN_PATH', 'XDG_DATA_DIRS')


class DesktopEnvironmentTests(unittest.TestCase):
    def clean_environment(self, overrides):
        env = dict(os.environ)
        for name in VARIABLES:
            env.pop(name, None)
        env.update(overrides)
        code = 'import json, os; print(json.dumps({k: os.environ.get(k) for k in ' + repr(VARIABLES) + '}))'
        result = subprocess.run(
            ['bash', '-euc', 'source "$1"; source "$1"; exec "$2" -c "$3"',
             'puma-environment-check', str(GUARD), sys.executable, code],
            env=env, capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def test_removes_snap_gtk_and_user_cache_paths(self):
        cleaned = self.clean_environment({
            'GTK_PATH': '/snap/code/999/usr/lib/gtk-3.0',
            'GIO_MODULE_DIR': '/home/test/snap/code/common/.cache/gio-modules',
        })
        self.assertIsNone(cleaned['GTK_PATH'])
        self.assertIsNone(cleaned['GIO_MODULE_DIR'])

    def test_preserves_ros_and_system_entries(self):
        cleaned = self.clean_environment({
            'LD_LIBRARY_PATH': '/opt/ros/humble/lib:/snap/code/999/lib:/usr/local/lib',
            'QT_PLUGIN_PATH': '/snap/code/999/qt:/opt/ros/humble/plugins',
            'XDG_DATA_DIRS': '/home/test/snap/code/999:/usr/local/share:/usr/share',
        })
        self.assertEqual(cleaned['LD_LIBRARY_PATH'], '/opt/ros/humble/lib:/usr/local/lib')
        self.assertEqual(cleaned['QT_PLUGIN_PATH'], '/opt/ros/humble/plugins')
        self.assertEqual(cleaned['XDG_DATA_DIRS'], '/usr/local/share:/usr/share')

    def test_non_snap_custom_paths_are_unchanged(self):
        values = {'GTK_PATH': '/opt/custom gtk/modules', 'LD_LIBRARY_PATH': '/opt/ros/humble/lib',
                  'QT_PLUGIN_PATH': '/opt/custom/qt', 'XDG_DATA_DIRS': '/usr/share:/usr/local/share'}
        cleaned = self.clean_environment(values)
        for name, value in values.items():
            self.assertEqual(cleaned[name], value)

    def test_unset_variables_stay_unset(self):
        self.assertTrue(all(value is None for value in self.clean_environment({}).values()))
