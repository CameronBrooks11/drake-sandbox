from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import swift
import numpy as np
import time

# Define a 4-DOF DH model
robot = DHRobot(
    [
        RevoluteDH(a=0, alpha=np.pi / 2, d=0.3),  # link1
        RevoluteDH(a=0.265, alpha=0, d=0),  # link2
        RevoluteDH(a=0.265, alpha=0, d=0),  # link3
        RevoluteDH(a=0.150, alpha=0, d=0),  # link4 (wrist)
    ],
    name="harvest_arm",
)

# Launch Swift viewer
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

# Animate a trajectory
q0 = np.radians([0, 60, -90, 0])
q1 = np.radians([30, 80, -40, -20])
traj = robot.jtraj(q0, q1, 50)

for q in traj.q:
    robot.q = q
    env.step(0.02)
    time.sleep(0.02)
