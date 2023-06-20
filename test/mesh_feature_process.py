import open3d as o3d
import numpy as np
import matplotlib.cm as cm


# Load a mesh or any other 3D object
mesh = o3d.io.read_triangle_mesh("/workspaces/data/gibson/gibson_fullplus/Akiak/mesh.obj")

mesh.paint_uniform_color([0, 1, 1]) # RGB: Blue

# Convert to point cloud
pcd = mesh.sample_points_uniformly(number_of_points=500000)

# Create voxel grid from the point cloud
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)

# Get voxel centers
voxel_centers = np.array([voxel.grid_index*voxel_grid.voxel_size for voxel in voxel_grid.get_voxels()])

# Convert voxel centers to point cloud
pcd.points = o3d.utility.Vector3dVector(voxel_centers)

# Define rotation matrix for a 90-degree rotation around x-axis
rotation_matrix = np.array([[1, 0, 0],
                            [0, 0, -1],
                            [0, 1, 0]], dtype=np.float64)

# Apply rotation
pcd.rotate(rotation_matrix)

# Convert to numpy array
points = np.asarray(pcd.points)

z_cut_height=1
# Calculate the z-coordinate that is 1 meter below the maximum
z_cut = np.max(points[:, 2]) - z_cut_height

print(z_cut)

# Create a mask for points below this z-coordinate
mask = points[:, 2] < z_cut

# Create a new point cloud with only the points below this z-coordinate
pcd_cut = o3d.geometry.PointCloud()
pcd_cut.points = o3d.utility.Vector3dVector(points[mask])

# Create a visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add geometries to visualizer
# vis.add_geometry(mesh)
vis.add_geometry(pcd_cut)

# Set background color to black
opt = vis.get_render_option()
opt.background_color = np.asarray([1, 1, 1]) # RGB: Black

# Draw the geometries
vis.run()

# Close the visualizer
vis.destroy_window()