#!/usr/bin/env python3
"""Source-tree entrypoint; ROS installations use the package console script."""
from pathlib import Path
import sys

# Keep direct execution from scripts/ working before the ROS package is built.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_pegasus_isaac_sim.stream_video_udp import main


if __name__ == '__main__':
    main()
