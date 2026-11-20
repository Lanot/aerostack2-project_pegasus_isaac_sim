#!/usr/bin/env python3
"""Isaac Sim 6.x camera/LiDAR drone example with a following viewport camera.

Based on Marcelo Jacinto's 8_camera_vehicle.py (BSD-3-Clause, 2024).
"""
from project_pegasus_isaac_sim.scripts.isaacsim6_app import main

if __name__ == '__main__':
    main(following=True)
