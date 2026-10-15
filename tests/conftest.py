from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
for folder in ("experiments/ik", "offline_kinematics", "tools"):
    sys.path.insert(0, str(ROOT / folder))
