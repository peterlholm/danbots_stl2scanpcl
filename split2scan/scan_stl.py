"""
convert stl file to pcl
Scan pcl from different position
generate new pointclods
"""

from copy import deepcopy
from pathlib import Path
import open3d as o3d
import numpy as np


POINT_PR_M2 = 160 * 160 * 100 * 100
FULL_RES_POINTS = 1000000
PAINT_COLOR = True
MAX_DISTANCE = 0.135

DEBUG = False

def point_distance(p1, p2):
    "compute distance between 2 points"
    dist = np.linalg.norm(np.array(p1)-np.array(p2))
    print ("dist ", dist)

def remove_point_distance(pcl, pkt, max_dist=1):
    "Remove all pont in pointcloud where distance > max_dist"
    npkt = np.asarray(pkt)
    points = np.asarray(pcl.points)
    pcl1 = o3d.geometry.PointCloud()
    distarr = np.linalg.norm(points - npkt, axis=1)
    pcl1.points = o3d.utility.Vector3dVector(points[distarr <= max_dist ])
    if pcl.has_colors():
        colors = np.asarray(pcl.colors)
        pcl1.colors = o3d.utility.Vector3dVector(colors[distarr <= max_dist ])
    if DEBUG:
        print(f"removed {len(pcl.points) - len(pcl1.points)} points")
    return pcl1

def show_mesh(meshlist, axis=True, name="showmesh"):
    "Show Mesh"
    #meshlist = [mesh]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        meshlist.append(axis_pcd)
    for m in meshlist:
        if not m.has_triangle_normals():
            m.compute_triangle_normals()
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600)
                                # zoom=_ZOOM,
                                # front=_FRONT,
                                # lookat=_LOOKAT,
                                # up=_UP,
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)

def show_pcl(meshlist, axis=True, name="showpcl"):
    "Show pcl"
    for p in meshlist:
        if not p.has_normals():
            p.estimate_normals()
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, -0.0, 0.0])
        meshlist.append(axis_pcd)
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600)
 
def show_pcl2(meshlist, axis=True, name="showpcl", position=None):
    "Show pcl"
    for p in meshlist:
        if not p.has_normals():
            p.estimate_normals()
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, -0.0, 0.0])
        meshlist.append(axis_pcd)
    if position is None:
        position=(0,0,5)
    
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600,
                                zoom=0.7,
                                front=position,
                                lookat=(0,0,0),
                                up=(0,1,0))
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                # mesh_show_wireframe=False)

def center_mesh(mesh):
    "center the mesh around 0,0,0"
    new = mesh.translate((0,0,0), relative=False)
    return new

def stl2pcl(stl, number=None, method="uniformly"):
    "convert mesh to pointcloud with number samples, number defaults to 160x160 points pr cm2"
    area = stl.get_surface_area()
    if number is None:
        no_points = int(POINT_PR_M2 * area)
    else:
        no_points = number
    print("Surface area", area, "no points", no_points)
    if method=="uniformly":
        pcl = stl.sample_points_uniformly(no_points)
    elif method=="poisson":
        pcl = stl.sample_points_poisson_disk(no_points)
    else:
        raise NotImplementedError("stl2pcl must be call with method==poisson or uninformly")
    return pcl

def hide_point(pcl, camera=(0,-0.1, 0), radius=10.1):
    "hide points seen from camera"
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
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
    color = [ [0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1],
                [0.5,0,0],[0.5,0,1],[0.5,1,0],[0.5,1,1],[0.5,0,0],[0.5,0,0.5],[0.5,0.5,0],[0.5,0.5,0.5],
                [0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1],
                [0.5,0,0],[0.5,0,1],[0.5,1,0],[0.5,1,1],[0.5,0,0],[0.5,0,0.5],[0.5,0.5,0],[0.5,0.5,0.5],
    ]
    nr=1
    opcl = stl2pcl(mesh)
    o3d.io.write_point_cloud(str(Path(filename).parent / 'ORG/center.ply'), opcl)
    for p in positions:
        pcl = deepcopy(opcl)
        p_view = hide_point(pcl, camera=p)
        #show_pcl(p_view, name="camera "+str(p))
        filen = filename + str(nr) + ".ply"
        file2 = filename + str(nr) + "_1.ply"
        o3d.io.write_point_cloud(filen, p_view)
        pclA = transform_pcl(p_view, p)
        o3d.io.write_point_cloud(file2, pclA)
        print("camera position", p)
        newpcl = remove_point_distance(p_view, p, max_dist=MAX_DISTANCE)
        if PAINT_COLOR:
            newpcl.paint_uniform_color(color[nr-1])
        if DEBUG:
            show_pcl2([p_view, newpcl], name="camera "+str(p), position=p)
        filen = f"{filename}{nr:02}.ply"
        o3d.io.write_point_cloud(filen, p_view)
        print("Number of points", len(p_view.points))
        nr += 1

if __name__=="__main__":
    INFIL = Path('testdata/stl/tooth/two_tooth/fortand_r.stl')
    OUTPATH = Path(__file__).parent.parent / "tmp"
    print(OUTPATH)
    mymesh = o3d.io.read_triangle_mesh(str(INFIL))
    cmesh = center_mesh(mymesh)

    (OUTPATH / 'ORG').mkdir(exist_ok=True)
    cmesh.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(OUTPATH / 'ORG' / 'center.stl'), cmesh)

    if DEBUG:
        orgpcl = stl2pcl(cmesh, 1000000)
        show_mesh([cmesh], name="Centered mesh")
        #show_pcl(orgpcl, name="original PCL")
        print(f"size  min: {orgpcl.get_min_bound()} Max: {orgpcl.get_max_bound()}")
        o3d.io.write_point_cloud('testdata/stl/tooth/two_tooth/fortand.ply', orgpcl)

    dx= 0.006
    Y=-0.000
    dy=-0.01
    Z = 0.019
    dz= -0.007
    y=-0.015
    d=0.01
    mypositions = [(-dx*2, Y, Z), (-dx, Y, Z),  (0.00, Y, Z), (dx, Y, Z), (2*dx, Y, Z), # 1-5
                    (-dx*2, Y+dy, Z), (-dx, Y+dy, Z),  (0.00, Y+dy, Z), (+dx, Y+dy, Z), (2*dx, Y+dy, Z),
                    (-dx*2, Y+dy, Z+dz), (-dx, Y+dy, Z+dz),  (0.00, Y+dy, Z+dz), (+dx, Y+dy, Z+dz), (2*dx, Y+dy, Z+dz),
                    (-dx*2, Y+2*dy, Z+2*dz), (-dx, Y+2*dy, Z+2*dz),  (0.00, Y+2*dy, Z+2*dz), (+dx, Y+2*dy, Z+2*dz), (2*dx, Y+2*dy, Z+2*dz),
                    (-dx*2, Y+dy, Z+3*dz), (-dx, Y+dy, Z+3*dz),  (0.00, Y+dy, Z+3*dz), (+dx, Y+dy, Z+3*dz), (2*dx, Y+dy, Z+3*dz),
                    (-dx*2, Y, Z+3*dz), (-dx, Y, Z+3*dz),  (0.00, Y, Z+3*dz), (+dx, Y, Z+3*dz), (2*dx, Y, Z+3*dz),
                  ]
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
