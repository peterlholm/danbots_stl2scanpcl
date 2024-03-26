"""
convert stl file to pcl
Scan pcl from different position
generate new pointclods
"""

from copy import deepcopy
from pathlib import Path
import open3d as o3d

DEBUG = False
def show_mesh(mesh, axis=True, name="showmesh"):
    "Show Mesh"
    meshlist = [mesh]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        meshlist.append(axis_pcd)

    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600)
                                # zoom=_ZOOM,
                                # front=_FRONT,
                                # lookat=_LOOKAT,
                                # up=_UP,
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)
def show_pcl(pcl, axis=True, name="showpcl"):
    "Show pcl"
    meshlist = []
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, -0.0, 0.0])
        meshlist.append(axis_pcd)

    if not pcl.has_normals():
        pcl.compute_normals()
    meshlist.append(pcl)
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600)
                                # zoom=_ZOOM,
                                # front=_FRONT,
                                # lookat=_LOOKAT,
                                # up=_UP,
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)

def center_mesh(mesh):
    "center the mesh around 0,0,0"
    new = mesh.translate((0,0,0), relative=False)
    return new

def stl2pcl(stl):
    "convert mesh to pointcloud"
    pcl = stl.sample_points_uniformly(10000)
    return pcl

def hide_point(pcl, camera=(0,-0.1, 0), radius=1.1):
    "hide points seen from camera"
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
    if DEBUG:
        show_pcl(pcd, name="HIDE " + str(camera))
    return pcd

def transform_pcl(pcl, position):
    "transform pointcloud to look as camera"
    print("Position", position)
    transform = (-position[0],-position[1],-position[2])
    pcl1 = deepcopy(pcl)
    print(pcl1.get_center())
    pcl1 = pcl1.translate(transform, relative=True)
    #o3d.io.write_point_cloud('tmp/trans1.ply', pcl1)
    return pcl1
    pcl2 = pcl.translate((0.01,0,0), relative=False)
    o3d.io.write_point_cloud('tmp/trans2.ply', pcl2)
    
def scan_mesh(mesh, positions, filename):
    "filter mesh from given positions"
    nr=1
    orgpcl = stl2pcl(mesh)
    for p in positions:
        pcl = deepcopy(orgpcl)
        #show_pcl(pcl)
        print("camera", p)
        p_view = hide_point(pcl, camera=p)
        #show_pcl(p_view, name="camera "+str(p))
        filen = filename + str(nr) + ".ply"
        file2 = filename + str(nr) + "_1.ply"
        o3d.io.write_point_cloud(filen, p_view)
        pclA = transform_pcl(p_view, p)
        o3d.io.write_point_cloud(file2, pclA)
        nr += 1

if __name__=="__main__":
    
    INFIL = Path('testdata/stl/tooth/two_tooth/fortand.stl')
    OUTPATH = INFIL.parent

    mymesh = o3d.io.read_triangle_mesh(str(INFIL))
    if DEBUG:
        show_mesh(mymesh, name="Original mesh")
    cmesh = center_mesh(mymesh)
    # if DEBUG:
    #     show_mesh(cmesh, name="Centered mesh")
    mypcl = stl2pcl(cmesh)
    if DEBUG:
        show_pcl(mypcl, name="original PCL")
        print("size", mypcl.get_min_bound(), mypcl.get_max_bound())
    o3d.io.write_point_cloud('testdata/stl/tooth/two_tooth/fortand.ply', mypcl)

    mypositions = [(0, -0.01, 0),(-0.01, -0.01, 0.0),(+0.01, -0.01, 0.0),(0, -0.00, -0.01),(-0.01, -0.00, -0.01),(+0.01, -0.00, -0.01),(0, -0.01, 0.0),(-0.01, -0.01, 0.0),(+0.01, -0.01, 0.0)]
    scan_mesh(cmesh, mypositions, "tmp/file")

    mymesh = o3d.io.read_point_cloud("tmp/file1.ply")
    transform_pcl(mymesh,(0,0,0))

    # o3d.io.write_point_cloud(str(OUTPATH / 'crop.ply')
    # pcl = stl2pcl(crop)
    # o3d.io.write_triangle_mesh('crop.stl', crop)
    # show_pcl(pcl)
    # hide_point(pcl)
    #mesh2ply(INFIL,OUTPATH / 'Bridge1.ply')
    #meshsurface2ply(INFIL, OUTPATH / 'fil1.ply', OUTPATH / 'fil2.ply')
