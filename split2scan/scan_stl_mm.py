"""
convert stl file to pcl
Scan pcl from different position
generate new pointclods
Workings in mm as unit
"""
#from math import sqrt
from copy import deepcopy
from pathlib import Path
from shutil import rmtree
import open3d as o3d
import numpy as np

# scanning resolution
# Omnicam: 79 True Definition: 54 Trios: 41 iTero: 34 point pr mm2
# point pr mm2 default to a point distance 1/16 = 0.062 mm
POINT_PR_MM2  = 256  # = 160 * 160 / 100
PAINT_COLOR = True

DEBUG = False

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
        print(f"removed distance {len(pcl.points) - len(pcl1.points)} points")
    return pcl1

def show_geo(meshlist, axis=True, name="show geometries"):
    "Show Mesh"
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0, origin=[0, 0, 0])
        meshlist.append(axis_pcd)
    for p in meshlist:
        if isinstance(p, o3d.geometry.PointCloud):
            if not p.has_normals():
                p.estimate_normals()
        elif isinstance(p, o3d.geometry.TriangleMesh):
            if not p.has_triangle_normals():
                p.compute_triangle_normals()
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600)

def show_geopos(meshlist, axis=True, name="showpcl2", position=None, lookat=None):
    "Show pcl"
    for p in meshlist:
        #print("Type", type(p))
        if isinstance(p, o3d.geometry.PointCloud):
            if not p.has_normals():
                p.estimate_normals()
        elif isinstance(p, o3d.geometry.TriangleMesh):
            if not p.has_triangle_normals():
                p.compute_triangle_normals()
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, -0.0, 0.0])
        meshlist.append(axis_pcd)
    if position is None:
        position=(0,0,5)
    if lookat is None:
        lookat = (0,0,0)
    print(f"Position {position} Lookat {lookat}")
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600,
                                zoom=0.7,
                                front=position,
                                lookat=lookat,
                                up=(0,1,0),
                                # point_show_normal=False,
                                # mesh_show_back_face=True,
                                mesh_show_wireframe=True
    )
def center_mesh(mesh):
    "center the mesh around 0,0,0"
    new = mesh.translate((0,0,0), relative=False)
    return new

def stl2pcl(stl, number=None, method="poisson"):
    "convert mesh to pointcloud with number samples, number defaults to POINT_PR_MM2"
    area = stl.get_surface_area()
    if number is None:
        no_points = int(POINT_PR_MM2 * area)
    else:
        no_points = number
    print("STL Surface area", area, "no points", no_points)
    if method=="uniformly":
        pcl = stl.sample_points_uniformly(no_points)
    elif method=="poisson":
        pcl = stl.sample_points_poisson_disk(no_points)
    else:
        raise NotImplementedError("stl2pcl must be call with method==poisson or uninformly")
    return pcl

def hide_point(pcl, camera=(0,-0.1, 0), radius=1):
    "hide points seen from camera"
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
    print(f"Hiding points from {len(pcl.points)} to {len(pt_map)}")
    #print("Hidden mesh type", type(mesh))
    #mesh.compute_triangle_normals()
    #show_geopos([mesh],name="hidden point mesh", position=(0,0,20))
    return pcd


def transform_pcl(pcl, position):
    "transform pointcloud to look as camera"
    transform = (-position[0],-position[1],-position[2])
    pcl1 = deepcopy(pcl)
    #print(pcl1.get_center())
    pcl1 = pcl1.translate(transform, relative=True)
    #o3d.io.write_point_cloud('tmp/trans1.ply', pcl1)
    return pcl1

def pos_pcl(pcl):
    "Move center to std position"
    stdpos = [0,0,-10]
    center = pcl.get_center()
    print("center", center)
    trans = stdpos - center
    print("Trans", trans)
    newpcl = pcl.translate(trans)
    return newpcl

def scan_mesh(mesh, positions, foldername, left_side=True):
    "filter mesh from given positions"

    color = [ [0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,0.7],                            # 1-8
                [0.5,0,0],[0.5,0,1],[0.5,1,0],[0.5,1,1],[0.5,0,0],[0.5,0,0.5],[0.5,0.5,0],[0.5,0.5,0.5],    # 2-16
                [0,0,0],[0,0,1],[0,1,0],[0,1,1],[1,0,0],[1,0,1],[1,1,0],[1,1,1],                            # 17-24
                [0.5,0,0],[0.5,0,1],[0.5,1,0],[0.5,1,1],[0.5,0,0],[0.5,0,0.5],[0.5,0.5,0],[0.5,0.5,0.5],    # 25-32
    ]
    colorleng = len(color)
    #axi = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=(0,0,0))
    foldername.mkdir(exist_ok=True)
    # clear old
    rmtree(foldername / "cut", ignore_errors = True)
    rmtree(foldername / "trans", ignore_errors = True)
    rmtree(foldername / "rot", ignore_errors = True)
    (foldername / "cut").mkdir(exist_ok=True)
    (foldername / "trans").mkdir(exist_ok=True)
    (foldername / "rot").mkdir(exist_ok=True)
    # picture see from front x right y up z front
    # cut: pcl from view in org coordinate
    # trans: pcl seen from viewpoint
    # rot: pcl seen with wand
    nr=1
    opcl = stl2pcl(mesh)
    o3d.io.write_point_cloud(str(Path(foldername) / 'ORG/center.ply'), opcl)
    for p in positions:
        print("Camera position", p)
        pcl = deepcopy(opcl)
        p_view = hide_point(pcl, camera=p, radius=1000.001)
        newpcl = remove_point_distance(p_view, p, max_dist=1500.0)
        if PAINT_COLOR:
            newpcl.paint_uniform_color(color[(nr-1) % colorleng])
        if DEBUG:
            #show_geopos([pcl], name="camera "+str(nr)+ " org " +str(p), position=p, lookat=(0,0,10))
            show_geopos([pcl, p_view, newpcl], name="camera "+str(nr)+ " all " +str(p), position=p, lookat=(0,0,10))
        filen = f"{foldername}/cut/file{nr:02}.ply"
        file2 = f"{foldername}/trans/file{nr:02}.ply"
        file3 = f"{foldername}/rot/file{nr:02}.ply"
        o3d.io.write_point_cloud(filen, newpcl)
        tr=transform_pcl(newpcl, p)
        o3d.io.write_point_cloud(file2, tr)
        #show_geo([tr], name="camera "+str(nr)+ " trans " +str(p))
        # convert to wand
        if left_side:
            r = tr.get_rotation_matrix_from_xyz((0,0,-np.pi/2))
            tr.rotate(r, center=p)
            o3d.io.write_point_cloud(file3, tr)
        #new = pos_pcl(tr)
        #show_geo([new])
        if DEBUG:
            show_geopos([tr], name="camera "+str(nr)+ " trans pos " +str(p), position=p, lookat=(0,0,-10))
        print("picture no", nr, "Number of points", len(p_view.points), len(newpcl.points))
        nr += 1

if __name__=="__main__":
    #INFIL = Path('testdata/stl/tooth/two_tooth/fortand_rr.stl')
    INFIL = Path('testdata/mm/fortand/fortand.cc.stl')
    OUTPATH = Path(__file__).parent.parent / "tmp"
    mymesh = o3d.io.read_triangle_mesh(str(INFIL))
    show_geo([mymesh], name="test")
    cmesh = center_mesh(mymesh)
    cmesh.compute_triangle_normals()
    rmtree(OUTPATH, ignore_errors = True)
    (OUTPATH / 'ORG').mkdir(exist_ok=True, parents=True)
    o3d.io.write_triangle_mesh(str(OUTPATH / 'ORG' / 'center.stl'), cmesh)

    if DEBUG:
        orgpcl = stl2pcl(cmesh)
        axi = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=(0,0,0))
        show_geo([cmesh, axi], name="Centered mesh")
        #show_geopos([cmesh, axi], name="Centered mesh2")
        print(f"Object size  min: {orgpcl.get_min_bound()} Max: {orgpcl.get_max_bound()}")
        o3d.io.write_point_cloud(str(INFIL.with_name("pointcloud.ply")), orgpcl)

    Zmax = cmesh.get_max_bound()[2]
    print ("Zmax", Zmax)
    Z = Zmax + 5    # 5 mm from tooth
    print("Z position", Z)
    mypositions = []
    # forfra
    Y = 0
    Z = Zmax + 5    # 5 mm from tooth
    for x in range(5):
        pos = ((x-2)*3, Y, Z)
        mypositions.append(pos)
    Y = -10
    Z = 5
    for x in range(5):
        pos = ((x-2)*3, Y, Z)
        mypositions.append(pos)
    Y = -5
    Z = -5
    for x in range(5):
        pos = ((x-2)*3, Y, Z)
        mypositions.append(pos)

    scan_mesh(cmesh, mypositions, OUTPATH )
