from pydrake.all import *
import numpy as np

# Build a simple 4-DoF serial arm (x-z plane links + yaw base) programmatically
builder = DiagramBuilder()
plant, scene = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

# Geometry + units
L = [0.30, 0.265, 0.265, 0.150]  # l1..l4 (m)

# Base
base = plant.AddRigidBody("base", SpatialInertia(0, [0,0,0], UnitInertia(1,1,1)))
X_WB = RigidTransform()  # at world origin

# Joints + links
parent = base
X = RigidTransform()
# J1: yaw about z at base
j1 = plant.AddJoint(RevoluteJoint("j1", parent, X, plant.world_body(), RigidTransform(), Axis.kZ))
parent = plant.world_body()  # after weld world<->base via j1 frame trick

# Add a welded vertical offset l1 from j1
body1 = plant.AddRigidBody("link1", SpatialInertia(0, [0,0,0], UnitInertia(1,1,1)))
plant.WeldFrames(plant.GetFrameByName("world"), body1.body_frame(), RigidTransform([0,0,L[0]]))

# Shoulder, elbow, wrist pitch (about y) with simple link frames
prev = body1
for i, li in enumerate(L[1:], start=2):
    link = plant.AddRigidBody(f"link{i}", SpatialInertia(0, [0,0,0], UnitInertia(1,1,1)))
    plant.WeldFrames(prev.body_frame(), link.body_frame(), RigidTransform([li,0,0]))
    plant.AddJoint(RevoluteJoint(f"j{i}", prev, RigidTransform(), link, RigidTransform(), Axis.kY))
    prev = link

# Visuals (capsules) so MeshCat shows links
for i, li in enumerate(L[1:], start=2):
    link = plant.GetBodyByName(f"link{i}")
    geom = geometry.GeometryInstance(RigidTransform(), geometry.Capsule(li/2, 0.02), f"g{i}")
    geom.set_illustration_properties(geometry.IllustrationProperties({"phong": {}}))
    scene.RegisterGeometry(plant.GetBodyFrameIdOrThrow(link.body_index()), geom)

plant.Finalize()

# MeshCat
meshcat = geometry.Meshcat()
visualizer = visualization.AddDefaultVisualization(builder, meshcat)

diagram = builder.Build()
context = diagram.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(context)

# Add joint sliders
sliders = manipulation.gui.SliderSystem(meshcat=meshcat)
for name in ["j1","j2","j3","j4"]:
    sliders.DeclareJoint(plant, name, lower=-np.pi, upper=np.pi, step=0.01)

# Publish once to open the viewer; then move joints programmatically
diagram.ForcedPublish(context)
q0 = [0, np.deg2rad(60), np.deg2rad(-90), 0]
plant.SetPositions(plant_context, q0)
diagram.ForcedPublish(context)
print("Open MeshCat URL:", meshcat.web_url())
