# -*- coding: utf-8 -*-
## I THINK THIS ONE IS THE ONE THAT WORKS
"""
Created on Mon Dec  2 17:38:31 2019

@author: monre
"""
'''
import subprocess

command1 = "meshlabserver -i 01.smooth_stls/femur_bone_smooth_9001104.stl -o 03.downsampled_PCDs/femur_bone_downsampled_9001104.ply -s 00.filters/clustering_decimation_4.mlx"
#command2 = "meshlabserver -i femur_bone_smooth_9002430.stl -o new/femur_bone_downsampled_9002430.ply -s clustering_decimation_4.mlx"
subprocess.call(command1, shell = True)
#subprocess.call(command2, shell = True)
'''

import subprocess
import os
directory = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/VCG/MeshLab/01.smooth_stls'
code_dir = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/VCG/MeshLab'
for filename in os.listdir(directory):
    os.chdir(code_dir)
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        command = "meshlabserver -i 01.smooth_stls/femur_bone_smooth_" + ID + ".stl"
        command = command + " -o 03.downsampled_PCDs2/femur_bone_downsampled_" + ID + ".ply" #NOTE:changing the ending from .ply to .stl saves them as STL files
        command = command + " -s 00.filters/clustering_decimation_meshlab_filter.mlx"
        # clustering_decimation_meshlab_filter.mlx gives around 130KB files, downsampling to approximately 3500 points
        subprocess.call(command, shell = True)
        
