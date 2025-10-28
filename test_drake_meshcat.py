# test_drake_meshcat.py — Drake 1.46.0, MeshCat 0.3.2, Python 3.11
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
import numpy as np, math, time

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Link lengths [l1 vertical, then planar x-links l2..l4]
L = [0.30, 0.265, 0.265, 0.150]

# Base
base = plant.AddRigidBody("base", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1)))
plant.RegisterVisualGeometry(
    base,
    RigidTransform(),
    Box(0.05, 0.05, 0.05),
    "base_vis",
    np.array([0.7, 0.2, 0.2, 1.0]),
)

# j1: yaw about z
plant.AddJoint(
    RevoluteJoint("j1", plant.world_frame(), base.body_frame(), np.array([0, 0, 1]))
)

# link1: fixed vertical offset
link1 = plant.AddRigidBody("link1", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1)))
plant.WeldFrames(base.body_frame(), link1.body_frame(), RigidTransform([0, 0, L[0]]))
plant.RegisterVisualGeometry(
    link1,
    RigidTransform([0, 0, L[0] / 2]),
    Box(0.03, 0.03, L[0]),
    "link1_vis",
    np.array([0.8, 0.3, 0.3, 1.0]),
)

# j2–j4: pitch about y
prev_body, prev_len = link1, 0.0
for i, li in enumerate(L[1:], start=2):
    link = plant.AddRigidBody(
        f"link{i}", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1))
    )
    if i == 2:
        parent_frame = prev_body.body_frame()
    else:
        parent_frame = plant.AddFrame(
            FixedOffsetFrame(
                f"{prev_body.name()}_end",
                prev_body.body_frame(),
                RigidTransform([prev_len, 0, 0]),
            )
        )
    plant.AddJoint(
        RevoluteJoint(f"j{i}", parent_frame, link.body_frame(), np.array([0, 1, 0]))
    )
    plant.RegisterVisualGeometry(
        link,
        RigidTransform([li / 2, 0, 0]),
        Box(li, 0.04, 0.04),
        f"link{i}_vis",
        np.array([0.3, 0.3 + 0.15 * (i - 2), 0.8 - 0.15 * (i - 2), 1.0]),
    )
    prev_body, prev_len = link, li

plant.Finalize()

# Visualization
meshcat = Meshcat()
cfg = VisualizationConfig()
cfg.publish_period = 0.02
ApplyVisualizationConfig(
    cfg, builder, plant=plant, scene_graph=scene_graph, meshcat=meshcat
)

diagram = builder.Build()
context = diagram.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(context)

# Initial pose
q0 = [0.0, np.deg2rad(45), np.deg2rad(-60), np.deg2rad(15)]
plant.SetPositions(plant_context, q0)
diagram.ForcedPublish(context)

# ---- MeshCat sliders ----
# Format: AddSlider(name, min, max, step, value)

meshcat.AddSlider("j1", -np.pi, np.pi, 0.01, q0[0])
meshcat.AddSlider("j2", -np.pi, np.pi, 0.01, q0[1])
meshcat.AddSlider("j3", -np.pi, np.pi, 0.01, q0[2])
meshcat.AddSlider("j4", -np.pi, np.pi, 0.01, q0[3])


print("Open URL:", meshcat.web_url())
print("Move sliders j1..j4 in MeshCat (Ctrl+C to quit).")


def safe_slider(name, default):
    """Clamp NaN or out-of-range slider values."""
    v = meshcat.GetSliderValue(name)
    return default if (v is None or math.isnan(v) or not np.isfinite(v)) else v


try:
    while True:
        q = [
            safe_slider("j1", q0[0]),
            safe_slider("j2", q0[1]),
            safe_slider("j3", q0[2]),
            safe_slider("j4", q0[3]),
        ]
        plant.SetPositions(plant_context, q)
        diagram.ForcedPublish(context)
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    meshcat.DeleteAddedControls()
