# -*- coding: utf-8 -*-
"""
Created on Thu Dec  5 18:28:01 2019

@author: monre
"""

import os
import open3d as o3d
import numpy as np
import pymesh

#registered = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_scaled_PCDs'
registered = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_PCDs'
#If you're looking at registered femurs without scaling, remove 9023193, 9131266, 9145695, 9291078, 9487462
#Calculates the maximum z-value per femur, and stores the smallest z-value of them all as min_z
min_z = 100
for filename in os.listdir(registered):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        if (ID != '9024940') and (ID != '9158391') and (ID != '9300338') and (ID != '9566781'): #These have a deformed and bent stem and we don't want these to affect the search for the minimum stem point
            femur_load = o3d.io.read_point_cloud(registered + '/femur_bone_registered_' + ID + '.ply')
            femur = np.asarray(femur_load.points)
            new = np.amax(femur, axis = 0)[2] #np.amax(femur, axis = 0) gives an array 1X3 with the values for max col0 (x), max col1, and max col2 - np.amax(femur, axis = 1) gives an array 1xn_cols with the first value being max row1 value, the second value being max row2 value,...
            if new < min_z:
                min_z = new
                shortest = ID

trimmed = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.trimmed_no_scaling_PCDs'
#trimmed_STL = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.trimmed_STLs'
for filename in os.listdir(registered):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        femur_load = o3d.io.read_point_cloud(registered + '/femur_bone_registered_' + ID + '.ply')
        femur = np.asarray(femur_load.points)
        femur = femur[femur[:,2].argsort(kind='mergesort')] #sorting row by third column (z)
        individual_max_z = np.amax(femur, axis = 0)[2]
        while individual_max_z > (min_z + 0.1):
            femur = np.delete(femur, -1, axis = 0)
            individual_max_z = np.amax(femur, axis = 0)[2]
        os.chdir(trimmed)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(femur)
        o3d.io.write_point_cloud('femur_bone_trimmed_' + ID + '.ply', pcd)
        #mesh = pymesh.load_mesh('femur_bone_trimmed_' + ID + '.ply')
        #os.chdir(trimmed_STL)
        #pymesh.save_mesh('femur_bone_trimmed_' + ID + '.stl', mesh);

minimum = 100000
min_height = 100
for filename in os.listdir(trimmed):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        femur_load = o3d.io.read_point_cloud(trimmed + '/' + filename)
        femur = np.asarray(femur_load.points)
        new = len(femur[:,1]) #number of rows in column 1 of the femur array
        if new < minimum:
            minimum = new
            filename_minimum = ID #ID of femur with the least number of points after all have been trimmed
            #ID shouldn't be 9024940, 9158391, 9300338 or 9566781 since they're all outliers, and this ID will be the
            #reference for the deformable registration...


print('ID of femur with the least number of points: ' + filename_minimum)
print('Number of points: ' + str(minimum))
print('ID of shortest femur: ' + shortest)

'''        
minimum = 1000000
maximum = 0
min_height = 100
for filename in os.listdir(trimmed):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        if (ID != '9024940') and (ID != '9158391') and (ID != '9300338') and (ID != '9566781'):
            femur_load = o3d.io.read_point_cloud(trimmed + '/' + filename)
            femur = np.asarray(femur_load.points)
            new = len(femur[:,1]) #number of rows in column 1 of the femur array
            if new < minimum:
                minimum = new
                filename_minimum = ID
            if new > maximum:
                maximum = new
                filename_maximum = ID
            height = femur[-1,2] #Gets the z-value of the last row of the component - This only works if rows have been sorted by z, and last z-value is max value
            if height < min_height:
                min_height = height
                shortest = ID
'''