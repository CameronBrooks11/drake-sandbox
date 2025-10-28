import meshcat
vis = meshcat.Visualizer()
vis.open()
print("MeshCat URL:", vis.url())
vis["box"].set_object(meshcat.geometry.Box([0.1, 0.2, 0.3]))
