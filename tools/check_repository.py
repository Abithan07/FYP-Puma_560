#!/usr/bin/env python3
"""Check source syntax and robot XML without ROS, NumPy, or a running simulator."""
import ast
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {'.git', '.venv', '.venv-ros', 'venv', 'build', 'install', 'log', '__pycache__'}


def main():
    counts = {'Python': 0, 'shell': 0, 'XML': 0}
    errors = []
    for path in sorted(ROOT.rglob('*')):
        if not path.is_file() or EXCLUDED.intersection(path.relative_to(ROOT).parts):
            continue
        try:
            if path.suffix == '.py':
                ast.parse(path.read_text(), filename=str(path))
                counts['Python'] += 1
            elif path.suffix == '.sh':
                result = subprocess.run(['bash', '-n', str(path)], capture_output=True, text=True)
                if result.returncode:
                    raise ValueError(result.stderr.strip())
                counts['shell'] += 1
            elif path.suffix in {'.xml', '.xacro', '.world', '.sdf'}:
                ET.parse(path)
                counts['XML'] += 1
        except (SyntaxError, ValueError, ET.ParseError, UnicodeError) as error:
            errors.append(f'{path.relative_to(ROOT)}: {error}')
    for error in errors:
        print(error, file=sys.stderr)
    print(', '.join(f'{count} {kind} files' for kind, count in counts.items()))
    return bool(errors)


if __name__ == '__main__':
    sys.exit(main())
