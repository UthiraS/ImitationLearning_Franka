import open3d as o3d
import numpy

filename = '/home/rbe07/MEPP2/build/out.ply'
# filename = '/home/rbe07/mer_lab/ros_ws/src/projects/active_vision/models/ycbAV/072-a_toy_airplane/google_16k/nontextured.ply'
mesh = o3d.io.read_point_cloud(filename)
print("Starting")
mesh.paint_uniform_color([1, 0.706, 0])
print(numpy.asarray(mesh.points))
o3d.visualization.draw_geometries([mesh],zoom=10,front=[0.4257, -0.2125, -0.8795],lookat=[2.6172, 2.0475, 1.532],up=[-0.0694, -0.9768, 0.2024])
print("done")