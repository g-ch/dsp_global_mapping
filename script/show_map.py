import open3d as o3d
import numpy as np

# Load the point cloud from the ply file
ply_file = "/home/cc/chg_ws/ros_ws/semantic_map_coda_ws/src/dsp_global_mapping/data/test_to_upload_higher_res/resampled_0-1/test_0.ply"
point_cloud = o3d.io.read_point_cloud(ply_file, format='ply')

# Load the labels from the txt file
label_file = "/home/cc/chg_ws/ros_ws/semantic_map_coda_ws/src/dsp_global_mapping/data/test_to_upload_higher_res/resampled_0-1/test_semantic_0.txt"
labels = np.loadtxt(label_file, dtype=int)

# Define the color map for labels
label_colors = {
    0: (0, 0, 0),
    1: (0, 0, 0),
    2: (0, 0, 0),
    3: (0, 0, 0),
    4: (0, 0, 0),
    5: (0, 74, 111),
    6: (81, 0, 81),
    7: (128, 64, 128),
    8: (232, 35, 244),
    9: (160, 170, 250),
    10: (140, 150, 230),
    11: (70, 70, 70),
    12: (156, 102, 102),
    13: (153, 153, 190),
    14: (180, 165, 180),
    15: (100, 100, 150),
    16: (90, 120, 150),
    17: (153, 153, 153),
    18: (0, 0, 0),
    19: (30, 170, 250),
    20: (0, 220, 220),
    21: (35, 142, 107),
    22: (152, 251, 152),
    23: (180, 130, 70),
    24: (60, 20, 220),
    25: (0, 0, 255),
    26: (142, 0, 0),
    27: (70, 0, 0),
    28: (100, 60, 0),
    29: (90, 0, 0),
    30: (110, 0, 0),
    31: (100, 80, 0),
    32: (230, 0, 0),
    33: (32, 11, 119),
    34: (128, 128, 64),
    35: (153, 153, 190),
    36: (90, 120, 150),
    37: (153, 153, 153),
    38: (64, 64, 0),
    39: (192, 128, 0),
    40: (0, 64, 128),
    41: (128, 64, 64)
}

# Create a numpy array of colors corresponding to labels
colors = np.array([label_colors[label] for label in labels], dtype=np.float64) / 255.0

# Assign the colors to the point cloud
point_cloud.colors = o3d.utility.Vector3dVector(colors)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])

