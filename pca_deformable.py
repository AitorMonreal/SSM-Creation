# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 18:18:29 2019

@author: monre
"""
#USED
import os
import open3d as o3d
import numpy as np
from numpy.core._multiarray_umath import ndarray

unscaled = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/06.corresponded_deformable_unscaled_PCDs'
#unscaled = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/06.corresponded_deformable_no_scaling_centred_PCDs'
pca_tests = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/07.pca_tests/deformable'

num_femurs = 0
for filename in os.listdir(unscaled):
    if filename[0:10] == 'femur_bone':
        num_femurs += 1

#We load the first femur to get some data, like len(femur)...
femur_load = o3d.io.read_point_cloud(unscaled + '/' + 'femur_bone_corresponded_unscaled_9001104.ply')
femur = np.asarray(femur_load.points)

i = 0
xyz_array = np.zeros([num_femurs, 3*len(femur)]) #one row per femur, with first the x-values for all its points, then the y-values, and then the z-values
for filename in os.listdir(unscaled):
    if filename[0:10] == 'femur_bone':
        ID = filename[-11:-4]
        femur_load = o3d.io.read_point_cloud(unscaled + '/' + filename)
        femur = np.asarray(femur_load.points)

        xyz_array[i, 0:len(femur)] = femur[:, 0] #x_array
        xyz_array[i, len(femur):2*len(femur)] = femur[:, 1] #y_array
        xyz_array[i, 2*len(femur):3*len(femur)] = femur[:, 2] #z_array
        i += 1

n = 100 #number of principal components
from sklearn.decomposition import PCA
pca = PCA(n_components=n)
pca.fit(xyz_array)
print(pca.explained_variance_ratio_) #the amount of variance explained by each principal component
print(pca.explained_variance_ratio_.cumsum()) #total variance explained with the addition of each principal component
print(pca.singular_values_)
print(pca.components_)
print(pca.mean_)
# TO SAVE THE DATA FOR THE RESULTS SECTION
'''
np.save('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_variance_ratio.npy', pca.explained_variance_ratio_)
np.savetxt('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_variance_ratio.csv', pca.explained_variance_ratio_, delimiter=",")
np.save('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_cumulative_variance_ratio.npy', pca.explained_variance_ratio_.cumsum())
np.savetxt('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_cumulative_variance_ratio.csv', pca.explained_variance_ratio_.cumsum(), delimiter=",")
'''
X_pca = pca.transform(xyz_array) #applies the pca transform to the data
#let's check the shape of X_pca array
#print(X_pca.shape)

#np.save('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/X_pca.npy', X_pca)

np.save(pca_tests+'/pca_components.npy', pca.components_)
np.save(pca_tests+'/pca_mean.npy', pca.mean_)
np.save(pca_tests+'/pca_variance.npy', pca.explained_variance_ratio_)
h=0
'''
#Plot the first principal component vs the second
Xax=X_pca[:,0]
Yax=X_pca[:,1]
import matplotlib.pyplot as plt 
marker={0:'*',1:'o'}
alpha={0:.3, 1:.5}
fig,ax=plt.subplots(figsize=(7,5))
fig.patch.set_facecolor('white')
ax.scatter(Xax,Yax)
plt.xlabel("First Principal Component",fontsize=14)
plt.ylabel("Second Principal Component",fontsize=14)
plt.show()
'''
'''
#Plot feature1 vs feature2, or feature2 vs feature3,... - essentially all the points for the x-axis of landmark1 point for all the femurs vs all the x-axis points for landmark2 point for all the femurs,...
Xax = xyz_array[:, 2] #the first landmark seems to have outliers for both the x-axis, y-axis[:,2124] and z-axis[:,4247]
Yax = xyz_array[:, 3]
import matplotlib.pyplot as plt 
marker = {0: '*', 1: 'o'}
alpha = {0: .3, 1: .5}
fig, ax = plt.subplots(figsize=(7, 5))
fig.patch.set_facecolor('white')
ax.scatter(Xax, Yax)
plt.xlabel("Feature3", fontsize=12) #Make sure to change the labels for the specific Landmark number
plt.ylabel("Feature4", fontsize=12)
plt.show()
'''
'''
#Plot feature1 vs feature2 (eg) in light blue for all femurs, and plot the dimensionality reduced version of the points on top in a different colour
###
f_x = 0
f_y = 1
import matplotlib.pyplot as plt
X_new = pca.inverse_transform(X_pca) #same size as initial xyz_array, essentially a prediction of xyz_array from a reduced information
plt.scatter(xyz_array[:, f_x], xyz_array[:, f_y], c='C0', alpha=0.4, label='Original') #C0-first colour in series
plt.scatter(X_new[:, f_x], X_new[:, f_y], c='C1', alpha=0.7, label='Model Reconstruction') #Plotting the dimensionality reduced data for the same landmarked points
#plt.axis('equal');
plt.xlabel("Feature3", fontsize=12) #Make sure to change the labels for the specific Feature number
plt.ylabel("Feature4", fontsize=12)
plt.legend(fontsize=12)
plt.show()
'''
'''
#Re-Generate a femur from the dimensionally reduced data - from X_pca
x_new = X_new[:,0:len(femur)]
y_new = X_new[:,len(femur):(2*len(femur))]
z_new = X_new[:,(2*len(femur)):(3*len(femur))]

new_femur = np.zeros((len(femur),3))
new_femur[:,0] = x_new[0,:]
new_femur[:,1] = y_new[0,:]
new_femur[:,2] = z_new[0,:]

new_femur = np.asarray(new_femur)
os.chdir(pca_tests)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(new_femur)
o3d.io.write_point_cloud('femur_bone_pca_' + ID + '.ply', pcd)
###
'''

#Creates femurs for different standard deviations of specific principal components, letting the others fixed
PC = 1
for i in range(-40,41,2):
#For PC1, holding the other principal components fixed at 0:
    s1 = np.std(X_pca[:,PC])
    vector = np.zeros((1,n))
    vector[0,PC] = (i/10)*s1
    X_new = np.matmul(vector, np.transpose(np.linalg.pinv(pca.components_))) + np.transpose(pca.mean_)

    x_new = X_new[:,0:len(femur)]
    y_new = X_new[:,len(femur):(2*len(femur))]
    z_new = X_new[:,(2*len(femur)):(3*len(femur))]

    new_femur = np.zeros((len(femur),3))
    new_femur[:,0] = x_new[0,:]
    new_femur[:,1] = y_new[0,:]
    new_femur[:,2] = z_new[0,:]

    new_femur = np.asarray(new_femur)
    os.chdir(pca_tests)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(new_femur)
    o3d.io.write_point_cloud('principal_component_' + str(PC) + '/femur_bone2_pca_' + str((i+40)/2) + '.ply', pcd)

PC = 2
for i in range(-40,41,2):
#For PC1, holding the other principal components fixed at 0:
    s1 = np.std(X_pca[:,PC])
    vector = np.zeros((1,n))
    vector[0,PC] = (i/10)*s1
    X_new = np.matmul(vector, np.transpose(np.linalg.pinv(pca.components_))) + np.transpose(pca.mean_)

    x_new = X_new[:,0:len(femur)]
    y_new = X_new[:,len(femur):(2*len(femur))]
    z_new = X_new[:,(2*len(femur)):(3*len(femur))]

    new_femur = np.zeros((len(femur),3))
    new_femur[:,0] = x_new[0,:]
    new_femur[:,1] = y_new[0,:]
    new_femur[:,2] = z_new[0,:]

    new_femur = np.asarray(new_femur)
    os.chdir(pca_tests)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(new_femur)
    o3d.io.write_point_cloud('principal_component_' + str(PC) + '/femur_bone2_pca_' + str((i+40)/2) + '.ply', pcd)