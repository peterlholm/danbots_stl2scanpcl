from pathlib import Path
from matplotlib.patches import Patch
import numpy as np
from matplotlib import use, pyplot as plt
#from matplotlib import use

O3D = True
if O3D:
    import open3d as o3d
    
_DEBUG = False
_SHOW = True

def show_stl(path):
    if not path.exists():
        print("File not found", path)
        return
    mesh = o3d.io.read_triangle_mesh(str(path))
    if _SHOW:
        o3d.visualization.draw_geometries([mesh], window_name="input",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=False,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)
        
        

def pcl2mesh(filepath):
    "make a mesh stl file from the filepath"
    downsample = 0.1
    pcl =o3d.io.read_point_cloud(str(filepath))
    if _SHOW:
        o3d.visualization.draw_geometries([pcl], window_name="input",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
    if _DEBUG:
        outfile = filepath.with_suffix('.in.jpg')
        pcl2jpg(pcl, outfile)
        print("Number of points: ", len(pcl.points))
        print("Downsample the point cloud with a voxel of", downsample)
    downpcd = pcl.voxel_down_sample(voxel_size=downsample)
    if _DEBUG:
        print("Number of points in downsample: ", len(downpcd.points))
        pcl2jpg(downpcd, filepath.with_suffix('.down.jpg'))
        outfile = filepath.with_suffix('.down.ply')
        o3d.io.write_point_cloud(str(outfile), downpcd)
    if _SHOW:
        o3d.visualization.draw_geometries([downpcd],  window_name="downsample",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)

    estimate_normals(downpcd)
    if _SHOW:
        o3d.visualization.draw_geometries([downpcd], window_name="normals",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)

    mesh = bpa(downpcd)
    if _SHOW:
        o3d.visualization.draw_geometries([mesh],  window_name="mesh",
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=False,
                                  mesh_show_back_face=True,
                                  mesh_show_wireframe=True)

    o3d.io.write_triangle_mesh(str(filepath.with_suffix('.stl')), mesh, print_progress=False)

    # make filtered stl
    filtered = o3d.geometry.TriangleMesh.filter_smooth_simple(mesh, 5)
    filtered = filtered.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(filepath.with_suffix('.filtered.stl')), filtered, print_progress=False)


if __name__=="__main__":
    FILE = Path(__file__).parent / 'testdata/LJ3.stl'
    FILE = Path(__file__).parent / 'testdata/niels/test3 LowerJawScan.stl'

    print (FILE)
    show_stl(FILE)
    