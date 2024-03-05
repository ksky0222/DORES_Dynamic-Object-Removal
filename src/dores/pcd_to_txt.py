import os
import numpy as np
from pypcd import pypcd
import sys

source_dir = "/home/airlab/catkin_dores/"
if len(sys.argv) > 1:
	data_dir = source_dir + "src/DORES/map_data/" + sys.argv[1] 
	pcd_file_path = os.path.join(source_dir + "src/DORES/map_data/" + sys.argv[1], sys.argv[1] + "_original_map.pcd") 
	output_txt_file = os.path.join(data_dir, sys.argv[1] + "_original_map.txt")  
else:
        print("no argv")

pcd_data = pypcd.PointCloud.from_path(pcd_file_path)
xyz_intensity = np.asarray(pcd_data.pc_data[['x', 'y', 'z', 'intensity']])

np.savetxt(output_txt_file, xyz_intensity, fmt='%f %f %f %d')

print("txt file saved. path:", output_txt_file)
