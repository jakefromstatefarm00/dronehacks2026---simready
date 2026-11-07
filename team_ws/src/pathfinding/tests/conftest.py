"""Run from the pathfinding folder: python3 -m pytest -q tests  (no ROS needed)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
