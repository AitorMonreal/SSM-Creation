# -*- coding: utf-8 -*-
# THIS ONE IS THE ONE THAT WORKS
"""
Created on Fri Dec  6 16:58:12 2019

@author: monre
"""

from functools import partial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pycpd.rigid_registration as rigid_registration
import numpy as np
#import time
import open3d as o3d
import os
import shutil
from stl import mesh

#start_time = time.time()

def visualize(iteration, error, X, Y, ax):
    plt.cla()
    ax.scatter(X[:,0],  X[:,1], X[:,2], color='red', label='Target')
    ax.scatter(Y[:,0],  Y[:,1], Y[:,2], color='blue', label='Source')
    ax.text2D(0.87, 0.92, 'Iteration: {:d}\nError: {:06.4f}'.format(iteration, error), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes, fontsize='x-large')
    ax.legend(loc='upper left', fontsize='x-large')
    '''
    plt.draw()
    plt.pause(0.001)
    '''
      
def main(ID_target, ID_source):
    target_load = o3d.io.read_point_cloud('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_scaled_PCDs/femur_bone_registered_' + ID_target + '.ply')
    femur_target = np.asarray(target_load.points)

    source_load = o3d.io.read_point_cloud('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/VCG/MeshLab/03.downsampled_PCDs/femur_bone_downsampled_' + ID_source + '.ply')
    femur_source = np.asarray(source_load.points)
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    callback = partial(visualize, ax=ax)
    '''
    reg = rigid_registration(**{ 'X': femur_target, 'Y':femur_source })

    #reg.register(callback)
    #plt.show()
    
    #return reg.register(callback)[0]
    #return reg.register(callback)[2]
    
    dictionary = {"femur": reg.register()[0],
                  "scaling": reg.register()[2]}
    return dictionary
    

downsampled = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/VCG/MeshLab/03.downsampled_PCDs'
registered = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/04.registered_scaled_PCDs'

#ADD THE FIRST FEMUR ONTO THE FOLDER, TO USE AS REFERENCE FOR ALIGNMENT OF THE REST
target_load = downsampled + '/femur_bone_downsampled_9001104.ply'
shutil.copy(target_load, registered + '/femur_bone_registered_9001104.ply')

#your_mesh = mesh.Mesh.from_file('some_file.stl')

#REGISTRATION
ID_list = []
scaling_list = []
i = 0 #number of files read
for filename in os.listdir(downsampled)[204:]:
    j = 0 #reference for registration, starting with 9001104 for all
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        #np.insert(ID_list, i, ID)
        ID_list.insert(i, ID) #add ID to list of registered IDs

        dictionary = main(ID_list[j], ID)
        aligned_femur = dictionary['femur']
        scaling = dictionary['scaling'] #factor by which the femur gets scaled when getting registered to reference
        while np.ndim(aligned_femur) == 0: #while it has returned 0, i.e. a scalar output
            j += 1 #try with the next reference
            if j == i: #if the next reference is the file you're trying to register, it hasn't been able to register with any of the already registered files. Problem -> ERROR and break
                print('error')
                break
            #aligned_femur = main(ID_list[j], ID)
            dictionary = main(ID_list[j], ID)
            aligned_femur = dictionary['femur'] #first return from main()
            scaling = dictionary['scaling'] #second return from main()
            
        scaling_list.insert(i, scaling)
        i += 1

        os.chdir(registered)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(aligned_femur)
        o3d.io.write_point_cloud('femur_bone_registered_' + ID + '.ply', pcd)


#SAVE scaling list TO text file, scaling.txt
with open(registered + '/scaling204_end.txt', 'w') as f:
    for item in scaling_list:
        f.write("%s\n" % item)
        
# To change registration parameters go to C:\Users\monre\Anaconda3\Lib\site-packages\pycpd
# C:\Users\monre\AppData\Local\Programs\Python\Python37\Lib\site-packages\pycpd

#print("--- %s seconds ---" % (time.time() - start_time))
