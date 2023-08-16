"Scan pointcloud file from a number of positions"
from pathlib import Path
import copy
import math
import numpy as np
import open3d as o3d

_DEBUG = False

def show_mesh(mesh, axis=True):
    "Show Mesh"
    meshlist = [mesh]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
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
def show_pcl(pcls, axis=True, name=""):
    "Show pcl"
    meshlist = []
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, -0.0, 0.0])
        meshlist.append(axis_pcd)
    for pcl in pcls:
        if type(pcl) == o3d.geometry.PointCloud:
            if not pcl.has_normals():
                pcl.compute_normals()
        meshlist.append(pcl)
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600)

def center_mesh(obj):
    "center the mesh around 0,0,0"
    new = obj.translate((0,0,0), relative=False)
    return new

def stl2pcl(stl):
    "convert mesh to pointcloud"
    pcl = stl.sample_points_uniformly(10000)
    return pcl

def hide_point(pcl, camera, radius=None):
    "hide points seen from camera"
    if radius is None:
        diameter = np.linalg.norm(np.asarray(pcl.get_max_bound()) - np.asarray(pcl.get_min_bound()))
        print("Diameter", diameter)
        radius = 100 * diameter
    #camera = [0,0,40]
    print("camera", camera, "radius", radius)
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
    return pcd

def translate(pcl, rotation_angles):
    "make a rotation of the object with rotation angles"
    rotation_radians = [rotation_angles[0] / 180 * math.pi, rotation_angles[1] / 180 * math.pi, rotation_angles[2] /180 * math.pi]
    r_matrix = o3d.geometry.PointCloud.get_rotation_matrix_from_axis_angle(rotation_radians)
    opcl = pcl.rotate(r_matrix, center=(0,0,0))
    return opcl

def scan_mesh(pcl, positions, filename):
    "filter mesh from given positions"
    nr=1
    for p in positions:
        #pcl = stl2pcl(mesh)
        #show_pcl(pcl)
        print("camera", p)
        p_view = hide_point(pcl, p, radius=1000*160)
        show_pcl(p_view, axis=True, name="position " + str(p))
        opcl = translate(p_view,[0,0,30])
        show_pcl(opcl, axis=True, name=" rot position " + str(p) )
        filen = filename + str(nr) + ".ply"
        o3d.io.write_point_cloud(filen, p_view)
        nr += 1

def get_scans(pcl, filename):
    "make a serie of scan files"
    positions = [(0,0,0),(45,0,0),(90,0,0),(135,0,0)]
    #camera = [0,0, 30]
    nr = 0
    for p in positions:
        nr += 1
        print(p)
        c_pcl = copy.deepcopy(pcl)
        rotation_angles = p
        print(rotation_angles)
        rotation_radians = [rotation_angles[0] / 180 * math.pi, rotation_angles[1] / 180 * math.pi, rotation_angles[2] /180 * math.pi]
        r_matrix = o3d.geometry.PointCloud.get_rotation_matrix_from_axis_angle(rotation_radians)
        opcl = c_pcl.rotate(r_matrix, center=(0,0,0))
        show_pcl([opcl], axis=True, name="rot position " + str(p))
        res = hide_point(opcl, [0,0,30])
        filen = filename + str(nr) + ".ply"
        o3d.io.write_point_cloud(filen, res)

def crop_pcl(pcl, center, size = 15):
    "Crop a box of size with center"
    hsize = size/2
    if _DEBUG:
        bounding_box = pcl.get_axis_aligned_bounding_box()
        print(bounding_box)
        bounding_box.color = (1,0,0)
    box = o3d.geometry.AxisAlignedBoundingBox([center[0]-hsize,center[1]-hsize,center[2]-hsize],[center[0]+hsize,center[1]+hsize,center[2]+hsize])
    if _DEBUG:
        box.color =(0,1,0)
        print(box)
        show_pcl([pcl, bounding_box, box], name="crop", axis=True)
    obox = pcl.crop(box)
    return obox


if __name__=="__main__":
    INFIL = Path('testdata/model/LowerJawScan_10k.ply')
    TAND_CENTER = (8, -7, 25.2)
    OUTPATH = Path("tmp")

    mypcl = o3d.io.read_point_cloud(str(INFIL))
    if _DEBUG:
        mypcl.paint_uniform_color([0.5,0.5,0.5])
    crop_pcl = crop_pcl(mypcl, TAND_CENTER)
    #crop_pcl.paint_uniform_color([1,0,0])
    show_pcl([mypcl, crop_pcl], name="crop", axis=True)
    o3d.io.write_point_cloud(str(OUTPATH / 'tand.ply'), crop_pcl)

    cpcl = center_mesh(crop_pcl)
    print(cpcl.get_axis_aligned_bounding_box())
    show_pcl([cpcl], axis=True, name="centeret")
    scans = [0]
    for p in scans:
        cp = copy.deepcopy(cpcl)
        cp.translate([0,p,0])
        get_scans(cp, "tmp/file_"+str(p)+"_")
