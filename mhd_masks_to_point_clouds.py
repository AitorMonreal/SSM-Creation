# -*- coding: utf-8 -*-
"""
Created on Mon Nov 18 21:24:22 2019

@author: monre
"""

import vtk
import os
import vtkplotter
import openmesh as om

rough_stls = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/00.rough_stls'
smooth_stls = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/01.smooth_stls'
point_clouds = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/processed_data/02.point_clouds'

directory = 'C:/Users/monre/OneDrive - Imperial College London/ME4/FYP/DATA/OAI-ZIB/segmentation_masks'
amount = 3 # 2-only femur bone; 3-femur bone & femur cartilage, ...
#TODO
for filename in os.listdir(directory):
    for i in range(1,amount):
        if i == 1:
            name = "femur_bone_"
        elif i == 2:
            name = "femur_cart_"
        elif i == 3:
            name = "tibia_bone_"
        else:
            name = "tibia_cart_"
        if filename.endswith(".mhd"):
            ID = filename[0:7]
            mhd_path = directory + "/" + filename
            reader = vtk.vtkMetaImageReader()
            reader.SetFileName(mhd_path)
            reader.Update()
            
            # Marching Cubes
            dmc = vtk.vtkDiscreteMarchingCubes()
            dmc.SetInputConnection(reader.GetOutputPort())
            # GenerateValues(int numContours, double rangeStart, double rangeEnd)
            dmc.GenerateValues(1, i, i) # To get the femur bone
            #dmc.GenerateValues(1, 1, 1) # To get the femur bone
            #dmc.GenerateValues(1, 2, 2) # To get the femur cartilage
            #dmc.GenerateValues(1, 3, 3) # To get the tibia bone 
            #dmc.GenerateValues(1, 4, 4) # To get the tibia cartilage
            #dmc.GenerateValues(4, 1, 4) # To get the full assembly 
            dmc.Update()
            
            # Create the stl file
            os.chdir(rough_stls) # Set directory
            writer = vtk.vtkSTLWriter()
            writer.SetInputConnection(dmc.GetOutputPort())
            writer.SetFileTypeToBinary()
            writer.SetFileName(name + ID + ".stl")
            writer.Write()
            
            # Load a mesh and show it
            a0 = vtkplotter.Plotter().load(name + ID + ".stl", threshold=True, c="v")
            #vp.show(a0, at=5)
            
            # Adjust mesh using Laplacian smoothing
            a0_lap = a0.smoothLaplacian(niter=50, relaxfact=0.1, edgeAngle=60, featureAngle=150).color("seagreen").alpha(1).legend("window sinc")
            # Adjust mesh using a windowed sinc function interpolation kernel
            a0_smooth = a0_lap.clone().smoothWSinc(niter=50, passBand=0.1, edgeAngle=15, featureAngle=90).color("crimson").alpha(1).legend("femur bone")
            os.chdir(smooth_stls) 
            #Save smoothened mesh
            vtkplotter.save(a0_smooth, name + "smooth_"+ ID + ".stl", binary = True)
            
            #Convert mesh to point cloud
            mesh = om.read_trimesh(name + "smooth_"+ ID + ".stl")
            os.chdir(point_clouds) 
            om.write_mesh(name + "cloud_"+ ID + ".ply", mesh)
            
'''
import open3d as o3d
pcd = o3d.io.read_point_cloud("femur_bone_cloud_9001104.ply")
o3d.visualization.draw_geometries([pcd])
'''


