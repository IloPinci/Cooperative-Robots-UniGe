import math
import socket
import struct
import time

import pybullet as p
import pybullet_data
from panda_robot import PandaRobot

INCLUDE_GRIPPER = True
SAMPLING_RATE = 1e-3  # Simulation step [s].
UDP_PORT = 1505

INITIAL_Q = [0.0167305, -0.762614, -0.0207622, -2.34352,
             -0.0305686, 1.53975, 0.753872]


def receive_positions(client_socket, positions):
    """Read the newest valid packet, keeping the last targets when MATLAB is silent."""
    while True:
        try:
            packet, _ = client_socket.recvfrom(4096)
        except BlockingIOError:
            return positions

        if len(packet) != 112:  # Fourteen doubles: seven joints per arm.
            continue
        received = struct.unpack('14d', packet)
        if all(math.isfinite(value) for value in received):
            positions = list(received)


def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        client_socket.bind(('127.0.0.1', UDP_PORT))
        client_socket.setblocking(False)

        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setTimeStep(SAMPLING_RATE)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf')
        left_arm = PandaRobot(False, include_gripper=INCLUDE_GRIPPER)
        right_arm = PandaRobot(True, include_gripper=INCLUDE_GRIPPER)

        # Initial camera: both arms centred in the view. Angles are in degrees.
        p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=50,
                                     cameraPitch=-30, cameraTargetPosition=[0.53, 0, 0.35])
        positions = INITIAL_Q * 2
        print('Viewer stays open after MATLAB finishes and holds the last joint targets.')
        print('Rotate: Ctrl + left drag. Pan: Ctrl + middle drag (press the wheel). Zoom: mouse wheel.')
        print('Close the window or press Ctrl+C in this terminal to exit.')

        while p.isConnected():
            positions = receive_positions(client_socket, positions)
            left_arm.set_target_positions(positions[:7])
            right_arm.set_target_positions(positions[7:])
            p.stepSimulation()
            time.sleep(SAMPLING_RATE)
    except KeyboardInterrupt:
        pass
    finally:
        client_socket.close()
        if p.isConnected():
            p.disconnect()


if __name__ == '__main__':
    main()
