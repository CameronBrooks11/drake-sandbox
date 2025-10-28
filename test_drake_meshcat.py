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

# --- Setup ---
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

L = [0.30, 0.265, 0.265, 0.150]  # l1 vertical, l2–l4 planar
base_h = 0.05  # base cube height

# --- Base cube flush on ground ---
base = plant.AddRigidBody("base", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1)))
plant.RegisterVisualGeometry(
    base,
    RigidTransform([0, 0, base_h / 2]),  # cube center, bottom at z=0
    Box(base_h, base_h, base_h),
    "base_vis",
    np.array([0.7, 0.2, 0.2, 1.0]),
)

# --- j1: yaw about Z at cube top ---
j1_frame = plant.AddFrame(
    FixedOffsetFrame("base_top", base.body_frame(), RigidTransform([0, 0, base_h / 2]))
)
plant.AddJoint(RevoluteJoint("j1", plant.world_frame(), j1_frame, np.array([0, 0, 1])))

# --- link1: vertical from j1 top ---
link1 = plant.AddRigidBody("link1", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1)))
plant.WeldFrames(j1_frame, link1.body_frame(), RigidTransform())
plant.RegisterVisualGeometry(
    link1,
    RigidTransform([0, 0, L[0] / 2]),
    Box(0.03, 0.03, L[0]),
    "link1_vis",
    np.array([0.8, 0.3, 0.3, 1.0]),
)

# --- shoulder frame at top of link1 ---
shoulder = plant.AddFrame(
    FixedOffsetFrame("shoulder", link1.body_frame(), RigidTransform([0, 0, L[0]]))
)

# --- j2–j4: planar pitch joints along +X ---
prev_frame = shoulder
for i, li in enumerate(L[1:], start=2):
    link = plant.AddRigidBody(
        f"link{i}", SpatialInertia(1, [0, 0, 0], UnitInertia(1, 1, 1))
    )
    plant.AddJoint(
        RevoluteJoint(f"j{i}", prev_frame, link.body_frame(), np.array([0, 1, 0]))
    )
    plant.RegisterVisualGeometry(
        link,
        RigidTransform([li / 2, 0, 0]),
        Box(li, 0.04, 0.04),
        f"link{i}_vis",
        np.array([0.3, 0.3 + 0.15 * (i - 2), 0.8 - 0.15 * (i - 2), 1.0]),
    )
    prev_frame = plant.AddFrame(
        FixedOffsetFrame(f"link{i}_end", link.body_frame(), RigidTransform([li, 0, 0]))
    )

plant.Finalize()

# --- Visualization ---
meshcat = Meshcat()
cfg = VisualizationConfig(publish_period=0.02)
ApplyVisualizationConfig(
    cfg, builder, plant=plant, scene_graph=scene_graph, meshcat=meshcat
)

diagram = builder.Build()
context = diagram.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(context)

# --- Initial pose ---
q0 = [0.0, np.deg2rad(45), np.deg2rad(-60), np.deg2rad(15)]
plant.SetPositions(plant_context, q0)
diagram.ForcedPublish(context)

# --- Sliders ---
for name, val in zip(["j1", "j2", "j3", "j4"], q0):
    meshcat.AddSlider(name, -np.pi, np.pi, 0.01, val)

print("Open URL:", meshcat.web_url())
print("Move sliders j1..j4 in MeshCat (Ctrl+C to quit).")


# --- Live loop ---
def safe_slider(name, default):
    v = meshcat.GetSliderValue(name)
    return default if (v is None or math.isnan(v) or not np.isfinite(v)) else v


try:
    while True:
        q = [safe_slider(j, q0[i]) for i, j in enumerate(["j1", "j2", "j3", "j4"])]
        plant.SetPositions(plant_context, q)
        diagram.ForcedPublish(context)
        time.sleep(0.02)
except KeyboardInterrupt:
    pass
finally:
    meshcat.DeleteAddedControls()
