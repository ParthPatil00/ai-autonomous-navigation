"""Script to execute simulation runs using reusable package entry points."""

import sys
from pathlib import Path

# Add src and project root to path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_ROOT / "src"
for p in [str(SRC_DIR), str(PROJECT_ROOT)]:
    if p not in sys.path:
        sys.path.insert(0, p)

from main import main

if __name__ == "__main__":
    sys.exit(main())
