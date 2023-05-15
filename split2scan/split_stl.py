"Split stl file in minor parts cload for stitching test"
from pathlib import Path
import open3d as o3d
# import colorsys
# from math import pi, sin ,cos
# import random
# import shutil
# import math

def show_mesh(mesh, axis=True):
    meshlist = [mesh]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, -0.01, 0.02])
        meshlist.append(axis_pcd)

    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    o3d.visualization.draw_geometries(meshlist, window_name="showcrop", width=800, height=600)  
                                # zoom=_ZOOM,
                                # front=_FRONT,
                                # lookat=_LOOKAT,
                                # up=_UP,
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)
def show_pcl(pcl, axis=True):
    meshlist = []
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, -0.01, 0.02])
        meshlist.append(axis_pcd)

    if not pcl.has_normals():
        pcl.compute_normals()
    meshlist.append(pcl)
    o3d.visualization.draw_geometries(meshlist, window_name="showcrop", width=800, height=600)  
                                # zoom=_ZOOM,
                                # front=_FRONT,
                                # lookat=_LOOKAT,
                                # up=_UP,
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)


def show_crop(mesh_f, cropbox, axis=True):
    "show th cropbox"
    if isinstance(mesh_f, (Path, str)):
        print("path")
        mesh = o3d.io.read_triangle_mesh(str(mesh_f))
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    meshlist = [mesh]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        meshlist.append(axis_pcd)
    box = o3d.geometry.TriangleMesh.create_box(0.01,0.01,0.01)
    meshlist.append(box)

    o3d.visualization.draw_geometries(meshlist, window_name="showcrop", width=800, height=600)  
                                # zoom=_ZOOM,
                                # front=_FRONT,
                                # lookat=_LOOKAT,
                                # up=_UP,
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)

def stl2pcl(stl):
    pcl = stl.sample_points_uniformly(10000)
    return pcl
def crop_stl(stl, crop):
    "Crop stl"
    minc = crop[0]
    maxc = crop[1]
    bbox = o3d.geometry.AxisAlignedBoundingBox(minc,maxc)
    box = stl.crop(bbox)
    #mesh = box.hidden_point_removal(camera, radius)
    show_mesh(box)
    return box


if __name__=="__main__":
    INFIL = Path('testdata/stl/tooth/Bridge1.stl')
    OUTPATH = INFIL.parent
    print(f"starting splitting {INFIL} to {OUTPATH}")
    mesh = o3d.io.read_triangle_mesh(str(INFIL))
    show_mesh(mesh)
    crop_area = ((-0.009,-1,-1), (0.012,1,1))
    crop = crop_stl(mesh, crop_area)
    o3d.io.write_point_cloud(str(OUTPATH / 'crop.ply')
    pcl = stl2pcl(crop)
    o3d.io.write_triangle_mesh('crop.stl', crop)
    show_pcl(pcl)
    hide_point(pcl)
    #mesh2ply(INFIL,OUTPATH / 'Bridge1.ply')
    #meshsurface2ply(INFIL, OUTPATH / 'fil1.ply', OUTPATH / 'fil2.ply')
