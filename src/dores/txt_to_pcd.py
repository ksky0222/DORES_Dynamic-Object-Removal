import os
import numpy as np
from pypcd import PointCloud
import sys

source_dir = "/home/airlab/catkin_dores/"
data_dir = source_dir + "src/DORES/map_data/" + sys.argv[1]
txt_file_path = os.path.join(data_dir, sys.argv[1] + "_dynamic_removed.txt") 
output_pcd_file = os.path.join(data_dir, sys.argv[1] + "_dynamic_removed.pcd")  

xyz_intensity = np.loadtxt(txt_file_path)

pcd_data = np.zeros(xyz_intensity.shape[0], dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
pcd_data['x'] = xyz_intensity[:, 0]
pcd_data['y'] = xyz_intensity[:, 1]
pcd_data['z'] = xyz_intensity[:, 2]
pcd_data['intensity'] = xyz_intensity[:, 3]

pcd = PointCloud.from_array(pcd_data)

with open(output_pcd_file, 'w') as f:
    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
    f.write("VERSION 0.7\n")
    f.write("FIELDS x y z intensity\n")
    f.write("SIZE 4 4 4 4\n")
    f.write("TYPE F F F F\n")
    f.write("COUNT 1 1 1 1\n")
    f.write("WIDTH " + str(len(pcd_data)) + "\n")
    f.write("HEIGHT 1\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS " + str(len(pcd_data)) + "\n")
    f.write("DATA ASCII\n")

    for point in pcd_data:
        f.write("{:f} {:f} {:f} {:.0f}\n".format(point['x'], point['y'], point['z'], point['intensity']))

print("PCD file saved. path:", output_pcd_file)

