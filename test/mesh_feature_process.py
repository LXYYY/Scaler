import torch
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.pyplot as plt


O3D_VIS = 1


def create_fov_lineset(cam_loc, cam_ori, fov_deg, d_near, d_far):
    # convert FOV to radians
    fov_rad = np.radians(fov_deg)

    # create a frustum using line segments
    points = [
        [0, 0, 0],
        [d_near * np.tan(fov_rad / 2), d_near * np.tan(fov_rad / 2), d_near],
        [-d_near * np.tan(fov_rad / 2), d_near * np.tan(fov_rad / 2), d_near],
        [-d_near * np.tan(fov_rad / 2), -d_near * np.tan(fov_rad / 2), d_near],
        [d_near * np.tan(fov_rad / 2), -d_near * np.tan(fov_rad / 2), d_near],
        [d_far * np.tan(fov_rad / 2), d_far * np.tan(fov_rad / 2), d_far],
        [-d_far * np.tan(fov_rad / 2), d_far * np.tan(fov_rad / 2), d_far],
        [-d_far * np.tan(fov_rad / 2), -d_far * np.tan(fov_rad / 2), d_far],
        [d_far * np.tan(fov_rad / 2), -d_far * np.tan(fov_rad / 2), d_far],
    ]

    lines = [
        [0, 1],
        [0, 2],
        [0, 3],
        [0, 4],
        [1, 2],
        [2, 3],
        [3, 4],
        [4, 1],
        [5, 6],
        [6, 7],
        [7, 8],
        [8, 5],
        [1, 5],
        [2, 6],
        [3, 7],
        [4, 8],
    ]

    colors = [[1, 0, 0] for i in range(len(lines))]  # color lines in red

    # create line set
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.array(points) + cam_loc),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # cam_ori quaternion is in [x, y, z, w] format
    # convert to rotation matrix
    cam_ori = np.array(cam_ori)
    cam_ori = np.roll(cam_ori, 1)
    cam_ori = o3d.geometry.get_rotation_matrix_from_quaternion(cam_ori)

    line_set.rotate(cam_ori, center=cam_loc)

    return line_set


# Load a mesh or any other 3D object
mesh = o3d.io.read_triangle_mesh("/workspaces/data/gibson/gibson_fullplus/Akiak/mesh.obj")

mesh.paint_uniform_color([0, 1, 1])  # RGB: Blue

# Convert to point cloud
pcd = mesh.sample_points_uniformly(number_of_points=500000)

# Create voxel grid from the point cloud
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)

# Get voxel centers
voxel_centers = np.array([voxel.grid_index * voxel_grid.voxel_size for voxel in voxel_grid.get_voxels()])

# Convert voxel centers to point cloud
pcd.points = o3d.utility.Vector3dVector(voxel_centers)

# Define rotation matrix for a 90-degree rotation around x-axis
rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)

# Apply rotation
pcd.rotate(rotation_matrix)

# get bounding box
bbox = pcd.get_axis_aligned_bounding_box()

# translate point cloud along z axis such that lowest z coordinate is 0
pcd.translate([-bbox.min_bound[0] / 2, -bbox.min_bound[1] / 2, -bbox.min_bound[2]], relative=False)

# compute centroid along x and y axes
centroid = np.mean(np.asarray(pcd.points), axis=0)

# translate point cloud along x and y axes such that centroid is at origin
pcd.translate([-centroid[0], -centroid[1], 0], relative=False)

min_z=np.min(np.asarray(pcd.points), axis=0)[2]
pcd.translate([0, 0, -min_z], relative=False)

# Convert to numpy array
points = np.asarray(pcd.points)

z_cut_height = 1
# Calculate the z-coordinate that is 1 meter below the maximum
z_cut = np.max(points[:, 2]) - z_cut_height

print(z_cut)

# Create a mask for points below this z-coordinate
mask = points[:, 2] < z_cut

# Create a new point cloud with only the points below this z-coordinate
pcd_cut = o3d.geometry.PointCloud()
pcd_cut.points = o3d.utility.Vector3dVector(points[mask])

print(len(pcd.points))

# Convert to PyTorch tensor
points_tensor = torch.from_numpy(points)

print(points_tensor.shape)  # Should print [N, 3] where N is the number of points

# Define acceptable range around z = 2
z_min = 2  # for example, 0.05
z_max = 6  # adjust these values as needed

# Filter the points to keep only those within the specified z range
slice_indices = (points_tensor[:, 2] > z_min) & (points_tensor[:, 2] < z_max)
slice_points = points_tensor[slice_indices]

# Plot the x and y coordinates of the points in the slice
plt.scatter(slice_points[:, 0], slice_points[:, 1], s=1)
plt.savefig("slice.png")

# a random robot position 3d with some noise
robot_pos = np.array([1, 0, 0.5]) 

# foward facing quaternion
robot_ori = np.array([0.707, 0, 0, 0.707])

fov = [60, 45]

if O3D_VIS:
    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Visualize robot_pos as a red sphere
    robot_pos_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    robot_pos_sphere.paint_uniform_color([1, 0, 0])  # RGB: Red
    robot_pos_sphere.translate(robot_pos)
    vis.add_geometry(robot_pos_sphere)

    # Add geometries to visualizer
    # set the color of the bbox to blue
    bbox.color = (0, 0, 1)
    # vis.add_geometry(bbox)
    vis.add_geometry(pcd_cut)

    # camera parameters
    fov_deg = 60  # field of view in degrees
    d_near = 1  # near distance
    d_far = 10  # far distance

    # create FOV lineset
    fov_lineset = create_fov_lineset(robot_pos, robot_ori, fov_deg, d_near, d_far)

    # visualize
    vis.add_geometry(fov_lineset)

    # Set background color to black
    opt = vis.get_render_option()
    opt.background_color = np.asarray([1, 1, 1])  # RGB: Black

    # Draw the geometries
    vis.run()

    # Close the visualizer
    vis.destroy_window()
