"""Check the supplied meshes and both robot models without opening a window."""
from pathlib import Path
import argparse
import sys

import numpy as np
import pybullet as p
import pybullet_data

sys.path.insert(0, str(Path(__file__).resolve().parent /
                       "simulator"))
from panda_robot import PandaRobot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Also check the OpenGL window.')
    args = parser.parse_args()
    client = p.connect(p.GUI if args.gui else p.DIRECT)
    assert client >= 0, 'Could not connect to PyBullet'
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        left = PandaRobot(False, include_gripper=True)
        right = PandaRobot(True, include_gripper=True)
        q = [0.0167305, -0.762614, -0.0207622, -2.34352,
             -0.0305686, 1.53975, 0.753872]
        left.set_target_positions(q)
        right.set_target_positions(q)
        for _ in range(100):
            p.stepSimulation()
        for arm in (left, right):
            positions, velocities = arm.get_position_and_velocity()
            assert np.isfinite(positions).all() and np.isfinite(velocities).all()
        assert p.getNumBodies() == 3
        assert np.allclose(right.get_base_position_and_orientation()[0], [1.06, -0.01, 0])
        print("PASS: NumPy, PyBullet, plane, both Panda models, meshes and simulation steps.")
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
