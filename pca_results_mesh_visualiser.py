import numpy as np
import open3d as o3d
import os
import trimesh
import pyglet


#Creates femurs for different standard deviations of specific principal components, letting the others fixed
PC = [0, 1, 2, 3]
sigmas = [-3, 0, 3]
X_pca = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/X_pca.npy')

pca_path = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/07.pca_tests/deformable'
#pca_mean = np.load(pca_path + '/pca_mean.npy')
#pca_components = np.load(pca_path + '/pca_components.npy')
pca_mean = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/pca_mean.npy')
pca_components = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/pca_components.npy')
pca_variance = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/pca_variance.npy')

faces = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/Depth_Camera_Data/Points_to_Surface/faces_connectivity_file.npy')
indexes = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/Depth_Camera_Data/Points_to_Surface/mesh_trimming_indexes.npy')
T = np.load('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/Depth_Camera_Data/Points_to_Surface/reordering_mesh_transformation.npy')

for i in range(len(PC)):
    for j in range(len(sigmas)):
    #For PC1, holding the other principal components fixed at 0:
        vector = np.zeros((1, 100))
        s = np.std(X_pca[:, PC[i]])
        # s = np.power(pca_variance[PC[i]], 0.5)  # this would be the square root of the eigenvalue, but it is far too small to visualise
        vector[0, PC[i]] = sigmas[j]*s
        X_new = np.matmul(vector, np.transpose(np.linalg.pinv(pca_components))) + np.transpose(pca_mean)

        model = np.zeros([int(X_new.size / 3), 3])
        model[:, 0] = X_new[0, 0:int(X_new.size / 3)]
        model[:, 1] = X_new[0, int(X_new.size / 3):int(2 * X_new.size / 3)]
        model[:, 2] = X_new[0, int(2 * X_new.size / 3):int(X_new.size)]

        trimmed_model = np.delete(model, indexes, axis=0)
        reordered_model = np.matmul(T, trimmed_model)
        mesh = trimesh.Trimesh(vertices=reordered_model,
                               faces=faces)
        # femur_vertices = np.asarray(mesh.vertices)
        mesh.export('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/femur_mode2_PC_' +
                    str(PC[i]) + '_s_' + str(sigmas[j]) + '.stl')
        #mesh.show()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(model)
        o3d.io.write_point_cloud('C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/code/SSM/Results/SSM_modes/femur_mode2_PC_' + str(PC[i]) + '_s_' + str(sigmas[j]) + '.ply', pcd)

