#! /usr/bin/env python3
"""Script to analyse pydocstyle linting errors in the package."""

import os
import unittest
import pydocstyle


def get_python_files():
    """Get all python files in the package."""
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    for root, _, files in os.walk(pkg_dir):
        for file in files:
            if file.endswith('.py') and \
                  file != 'setup.py' and file != '__init__.py':
                yield os.path.join(root, file)


class TestPyCodeStyle(unittest.TestCase):
    """Class for testing pydocstyle compliance."""

    def test_pydocstyle(self):
        """Test pydocstyle compliance."""
        failures = [str(fail) for fail in pydocstyle.check(get_python_files())]

        if failures:
            self.fail(f'Found {len(failures)} ' +
                      'pydocstyle failure(s):\n' + '\n'.join(failures))


if __name__ == "__main__":
    unittest.main()
