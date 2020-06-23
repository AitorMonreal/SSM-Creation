# -*- coding: utf-8 -*-
"""
Created on Fri Dec  6 16:58:12 2019

@author: monre
"""
#USED

#THIS CODE TAKES THE DOWNSAMPLED FEMURS AND FINDS THEIR CORRESPONDENCES

from functools import partial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pycpd.deformable_registration as deformable_registration
import numpy as np
import time
import open3d as o3d
import os
import shutil
from stl import mesh

def visualize(iteration, error, X, Y, ax):
    plt.cla()
    ax.scatter(X[:, 0], X[:, 1], X[:, 2], color='red', label='Target')
    ax.scatter(Y[:, 0], Y[:, 1], Y[:, 2], color='blue', label='Source')
    ax.text2D(0.87, 0.92, 'Iteration: {:d}\nError: {:06.4f}'.format(iteration, error), horizontalalignment='center',
              verticalalignment='center', transform=ax.transAxes, fontsize='x-large')
    ax.legend(loc='upper left', fontsize='x-large')
    '''
    plt.draw()
    plt.pause(0.001)
    '''
def main(ID_target, ID_source):

    target_load = o3d.io.read_point_cloud(
        'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.corresponded_deformable_no_scaling_PCDs/femur_bone_corresponded_deformable_' + ID_target + '.ply')
    femur_target = np.asarray(target_load.points)/8

    source_load = o3d.io.read_point_cloud(
        'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.trimmed_no_scaling_PCDs/femur_bone_trimmed_' + ID_source + '.ply')
    femur_source = np.asarray(source_load.points)/8

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    callback = partial(visualize, ax=ax)

    reg = deformable_registration(**{'X': femur_target, 'Y': femur_source})

    reg.register(callback)
    #plt.show()

    dictionary = {"femur": reg.register(callback)[0],
                  "probability": reg.register(callback)[2]}
    return dictionary


registered_trimmed = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.trimmed_PCDs'
deformable_corresponded = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/05.corresponded_deformable_scaled_PCDs'
# ADD THE FIRST FEMUR ONTO THE FOLDER, TO USE AS REFERENCE FOR ALIGNMENT OF THE REST
target_load = registered_trimmed + '/femur_bone_trimmed_9653075.ply'
shutil.copy(target_load, deformable_corresponded + '/femur_bone_corresponded_deformable_9653075.ply')

# REGISTRATION
#i = 0
#count = np.zeros([507, 1])
for filename in os.listdir(registered_trimmed)[300:]:
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        dictionary = main('9653075', ID)
        #aligned_femur = dictionary['femur']
        probability = dictionary['probability'] #get matrix of correspondence probabilities, with rows for source, and columns fo the reference/target - 2138 columns

        index = np.zeros([probability.shape[1], 1]) #1995-by-1 vector
        new_femur = np.zeros([probability.shape[1], 3]) #same size as the reference femur, 1995-by-3
        source_load = o3d.io.read_point_cloud(registered_trimmed + '/femur_bone_trimmed_' + ID +'.ply')
        femur_source = np.asarray(source_load.points)
        for row in range(index.size):
            index[row] = np.argmax(probability[:, row]) #For each reference point, get the index of the femur source point with the highest probability of being correspondent to it
            new_femur[row, :] = femur_source[int(index[row]), :]

        os.chdir(deformable_corresponded)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(new_femur)
        o3d.io.write_point_cloud('femur_bone_corresponded_deformable_' + ID + '.ply', pcd)
        '''
        index = np.sort(index, axis=None)  # sorting index
        for row in range(len(index) - 1):
            if index[row] == index[row + 1]:  # shouldn't only be
                count[i] += 1
        i+=1
        '''
#print(done)
