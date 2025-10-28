# test_drake_meshcat.py  — Drake 1.46.0, MeshCat 0.3.2, Python 3.11
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import (
    SpatialInertia,
    UnitInertia,
    RevoluteJoint,
    FixedOffsetFrame,
)
from pydrake.geometry import Meshcat, Box
from pydrake.visualization import VisualizationConfig, ApplyVisualizationConfig
from pydrake.math import RigidTransform
from pydrake.systems.framework import DiagramBuilder
import numpy as np

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Link lengths [l1 vertical, then planar x-links l2..l4] in meters
L = [0.30, 0.265, 0.265, 0.150]

# Base body and small visual
base = plant.AddRigidBody("base", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1)))
plant.RegisterVisualGeometry(
    base,
    RigidTransform(),
    Box(0.05, 0.05, 0.05),
    "base_vis",
    np.array([0.7, 0.2, 0.2, 1.0]),
)

# Joint 1: yaw about z (world ↔ base) at world origin
plant.AddJoint(
    RevoluteJoint(
        "j1", plant.world_frame(), base.body_frame(), np.array([0.0, 0.0, 1.0])
    )
)

# Link 1: fixed vertical offset from base by l1 along +Z
link1 = plant.AddRigidBody("link1", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1)))
plant.WeldFrames(base.body_frame(), link1.body_frame(), RigidTransform([0, 0, L[0]]))
# Visual for link1 centered along its own Z
plant.RegisterVisualGeometry(
    link1,
    RigidTransform([0, 0, L[0] / 2]),
    Box(0.03, 0.03, L[0]),
    "link1_vis",
    np.array([0.8, 0.3, 0.3, 1.0]),
)

# Joints 2–4: pitch about Y; each joint is at the *end* of its parent x-link
prev_body = link1
prev_len = 0.0  # link1 is vertical; j2 located at link1 origin
for i, li in enumerate(L[1:], start=2):
    link = plant.AddRigidBody(
        f"link{i}", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1))
    )

    # Parent joint frame: at parent end (offset by previous link length along +X).
    if i == 2:
        parent_frame = prev_body.body_frame()  # shoulder at link1 origin
    else:
        parent_frame = plant.AddFrame(
            FixedOffsetFrame(
                f"{prev_body.name()}_end",
                prev_body.body_frame(),
                RigidTransform([prev_len, 0, 0]),
            )
        )

    # Revolute joint about parent Y-axis
    plant.AddJoint(
        RevoluteJoint(
            f"j{i}", parent_frame, link.body_frame(), np.array([0.0, 1.0, 0.0])
        )
    )

    # Visual for this link: box of length li centered at li/2 on +X of the link
    plant.RegisterVisualGeometry(
        link,
        RigidTransform([li / 2, 0, 0]),
        Box(li, 0.04, 0.04),
        f"link{i}_vis",
        np.array([0.3, 0.3 + 0.15 * (i - 2), 0.8 - 0.15 * (i - 2), 1.0]),
    )

    prev_body = link
    prev_len = li

plant.Finalize()

# Visualization
meshcat = Meshcat()
cfg = VisualizationConfig()
cfg.publish_period = 0.05
ApplyVisualizationConfig(
    cfg, builder, plant=plant, scene_graph=scene_graph, meshcat=meshcat
)

diagram = builder.Build()
context = diagram.CreateDefaultContext()

# Initial pose [j1, j2, j3, j4] in radians
q0 = [0.0, np.deg2rad(45), np.deg2rad(-60), np.deg2rad(15)]
plant.SetPositions(plant.GetMyContextFromRoot(context), q0)
diagram.ForcedPublish(context)

import time

plant_context = plant.GetMyContextFromRoot(context)

# Animate joint motion
frames = 200
for t in range(frames):
    q = [
        np.deg2rad(15 * np.sin(2 * np.pi * t / frames)),  # base yaw
        np.deg2rad(45 + 15 * np.sin(2 * np.pi * t / frames)),  # shoulder
        np.deg2rad(-60 + 30 * np.sin(2 * np.pi * t / frames)),  # elbow
        np.deg2rad(15 * np.sin(4 * np.pi * t / frames)),  # wrist
    ]
    plant.SetPositions(plant_context, q)
    diagram.ForcedPublish(context)
    time.sleep(0.02)  # ~50 fps

print("Open URL:", meshcat.web_url())
input("Press Enter to exit …")
