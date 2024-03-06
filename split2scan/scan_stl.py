"""
convert stl file to pcl
Scan pcl from different position
generate new pointclods
"""

from copy import deepcopy
from pathlib import Path
import open3d as o3d

POINT_PR_M2 = 160 * 160 * 100 * 100
FULL_RES_POINTS = 1000000
PAINT_COLOR = True

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

def hide_point(pcl, camera=(0,-0.1, 0), radius=1.1):
    "hide points seen from camera"
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
    if DEBUG:
        show_pcl(pcd, name="HIDE " + str(camera))
    return pcd

def scan_mesh(mesh, positions, filename):
    "filter mesh from given positions"
    color = [ [0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1],
                [0.5,0,0],[0.5,0,1],[0.5,1,0],[0.5,1,1],[0.5,0,0],[0.5,0,0.5],[0.5,0.5,0],[0.5,0.5,0.5],
                [0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1],
                [0.5,0,0],[0.5,0,1],[0.5,1,0],[0.5,1,1],[0.5,0,0],[0.5,0,0.5],[0.5,0.5,0],[0.5,0.5,0.5],
    ]
    nr=1
    opcl = stl2pcl(mesh)
    for p in positions:
        pcl = deepcopy(opcl)
        if PAINT_COLOR:
            pcl.paint_uniform_color(color[nr-1])
        #show_pcl(pcl)
        print("camera", p)
        p_view = hide_point(pcl, camera=p)
        #show_pcl(p_view, name="camera "+str(p))
        filen = filename + str(nr) + ".ply"
        o3d.io.write_point_cloud(filen, p_view)
        print("Number of points", len(p_view.points))
        nr += 1

if __name__=="__main__":
    INFIL = Path('testdata/stl/tooth/two_tooth/fortand.stl')
    OUTPATH = INFIL.parent

    mymesh = o3d.io.read_triangle_mesh(str(INFIL))
    # if DEBUG:
    #     show_mesh(mymesh, name="Original mesh")
    cmesh = center_mesh(mymesh)
    if DEBUG:
        show_mesh(cmesh, name="Centered mesh")
    orgpcl = stl2pcl(cmesh, 1000000)
    if DEBUG:
        show_pcl(orgpcl, name="original PCL")
        print("size", orgpcl.get_min_bound(), orgpcl.get_max_bound())
    o3d.io.write_point_cloud('testdata/stl/tooth/two_tooth/fortand.ply', orgpcl)
    mypcl = stl2pcl(cmesh,1000)

    mypositions = [(0, -0.01, 0),(-0.005, -0.01, 0.0),(-0.01, -0.01, 0.0),
                   (+0.005, -0.01, 0.0),(+0.01, -0.01, 0.0),
                   (-0.01, -0.01, -0.02),(-0.005, -0.01, -0.02),(+0.0, -0.01, -0.02),(0.005, -0.01, -0.02),(0.01, -0.01, -0.02),
                   (-0.01, -0.005, -0.02),(-0.005, -0.005, -0.02),(+0.0, -0.005, -0.02),(0.005, -0.005, -0.02),(0.01, -0.005, -0.02),
                   (-0.01, -0.00, -0.02),(-0.005, -0.00, -0.02),(+0.0, -0.00, -0.02),(0.005, -0.00, -0.02),(0.01, -0.00, -0.02),
                   (-0.01, -0.00, -0.01),(-0.005, -0.00, -0.01),(+0.0, -0.00, -0.01),(0.005, -0.00, -0.01),(0.01, -0.00, -0.01),
                   ]
    scan_mesh(cmesh, mypositions, "tmp/file")

    # o3d.io.write_point_cloud(str(OUTPATH / 'crop.ply')
    # pcl = stl2pcl(crop)
    # o3d.io.write_triangle_mesh('crop.stl', crop)
    # show_pcl(pcl)
    # hide_point(pcl)
    #mesh2ply(INFIL,OUTPATH / 'Bridge1.ply')
    #meshsurface2ply(INFIL, OUTPATH / 'fil1.ply', OUTPATH / 'fil2.ply')
