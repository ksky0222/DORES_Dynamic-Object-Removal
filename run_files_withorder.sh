#!/bin/bash

#python src/dores/pcd_to_txt.py $1

#./build/devel/lib/DORES/my_cpp_executable_1 $1

./build/devel/lib/DORES/my_cpp_executable_2 $1

python src/dores/txt_to_pcd_withorder.py $1

python src/dores/analysis_withorder.py $1

