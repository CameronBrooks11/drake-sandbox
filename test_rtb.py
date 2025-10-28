import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb
from swift import Swift

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(a=0, alpha=np.pi / 2, d=0.3),
        rtb.RevoluteDH(a=0.265, alpha=0, d=0),
        rtb.RevoluteDH(a=0.265, alpha=0, d=0),
        rtb.RevoluteDH(a=0.150, alpha=0, d=0),
    ],
    name="harvest_arm",
)

env = Swift()
env.launch(realtime=True)
env.add(robot)

q0 = np.radians([0, 60, -90, 0])
q1 = np.radians([30, 80, -40, -20])
traj = robot.jtraj(q0, q1, 50)

for q in traj.q:
    robot.q = q
    env.step(0.02)
