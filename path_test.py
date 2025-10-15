#!/usr/bin/env python3
"""
Launcher for path_test.py application.

This is a convenience script that runs the main path testing application.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent / 'src'))

# Import and run the main application
from apps.path_test import main

if __name__ == "__main__":
    sys.exit(main())

