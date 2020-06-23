# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 13:47:18 2019

@author: monre
"""
#USED
import os
import open3d as o3d
import numpy as np

registered = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_scaled_PCDs'
corresponded = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.corresponded_deformable_scaled_PCDs'
#centered = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/06.corresponded_deformable_centered_PCDs'
unscaled = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/06.corresponded_deformable_unscaled_PCDs'

i = 0
scaling = []
# READ SCALING FACTOR AND CREATE SCALING LIST
with open('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_scaled_PCDs/scaling.txt') as f:
    for item in f:
        scaling.insert(i, float(item[:-1]))
        i += 1

origin = 0
i = 0
for filename in os.listdir(corresponded):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        # CENTER THE FEMURS ABOUT THE ORIGIN - Save to centered:
        femur_corresponded_load = o3d.io.read_point_cloud(corresponded + '/' + filename)
        femur_corresponded = np.asarray(femur_corresponded_load.points)
        if ID == '9001104':
            origin = np.mean(femur_corresponded, axis=0)
        femur_centered = femur_corresponded - origin
        '''
        os.chdir(centered)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(femur_centered)
        o3d.io.write_point_cloud('femur_bone_corresponded_centered_' + ID + '.ply', pcd)
        '''
        # REVERT SCALING FACTORS ON EACH FEMUR - Save to unscaled:
        femur_unscaled = femur_centered / scaling[i]
        i += 1
        os.chdir(unscaled)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(femur_unscaled)
        o3d.io.write_point_cloud('femur_bone_corresponded_unscaled_' + ID + '.ply', pcd)

'''
# REVERT SCALING FACTORS ON EACH FEMUR - Save to unscaled:
i = 0
for filename in os.listdir(centered):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        femur_centered_load = o3d.io.read_point_cloud(centered + '/' + filename)
        femur_centered = np.asarray(femur_centered_load.points)
        femur_unscaled = femur_centered / scaling[i]
        i += 1
        os.chdir(unscaled)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(femur_unscaled)
        o3d.io.write_point_cloud('femur_bone_corresponded_unscaled_' + ID + '.ply', pcd)
'''

# FOR CHECKING PURPOSES - Center the initial femurs too
'''
normal = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_centered_PCDs'
# CENTER THE FEMURS ABOUT THE ORIGIN - ON THE FEMURS THAT DON'T HAVE A POINT TO POINT CORRESPONDENCE AND HAVEN'T BEEN SCALED, TO MAKE SURE THEY'RE SIMILAR TO THE UNSCALED ONES LATER
femur_correspond_load = o3d.io.read_point_cloud(corresponded + '/femur_bone_corresponded_9001104.ply')
femur_correspond = np.asarray(femur_correspond_load.points)
origin = np.mean(femur_correspond, axis=0)
for filename in os.listdir(registered):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        femur_correspond_load = o3d.io.read_point_cloud(registered + '/' + filename)
        femur_correspond = np.asarray(femur_correspond_load.points)
        femur_correspond = femur_correspond - origin
        os.chdir(normal)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(femur_correspond)
        o3d.io.write_point_cloud('femur_bone_registered_centered_' + ID + '.ply', pcd)
'''
