"Scan stl file from a number of positions"
from pathlib import Path
import open3d as o3d


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
def show_pcl(pcl, axis=True):
    "Show pcl"
    meshlist = []
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, -0.0, 0.0])
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

def center_mesh(mesh):
    "center the mesh around 0,0,0"
    new = mesh.translate((0,0,0), relative=False)
    return new

def stl2pcl(stl):
    "convert mesh to pointcloud"
    pcl = stl.sample_points_uniformly(10000)
    return pcl

def hide_point(pcl, camera=(0,-0.1, 0), radius=100.1):
    "hide points seen from camera"
    _, pt_map = pcl.hidden_point_removal(camera, radius)
    pcd = pcl.select_by_index(pt_map)
    show_pcl(pcd)
    return pcd

def scan_mesh(mesh, positions, filename):
    "filter mesh from given positions"
    nr=1
    for p in positions:
        pcl = stl2pcl(mesh)
        #show_pcl(pcl)
        print("camera", p)
        p_view = hide_point(pcl, camera=p)
        show_pcl(p_view)
        filen = filename + str(nr) + ".ply"
        o3d.io.write_point_cloud(filen, p_view)
        nr += 1

if __name__=="__main__":
    INFIL = Path('testdata/stl/tooth/two_tooth/fortand.stl')
    OUTPATH = INFIL.parent

    mymesh = o3d.io.read_triangle_mesh(str(INFIL))
    #show_mesh(mesh)
    cmesh = center_mesh(mymesh)
    mypcl = stl2pcl(cmesh)
    o3d.io.write_point_cloud('testdata/stl/tooth/two_tooth/fortand.ply', mypcl)
    #show_mesh(cmesh)
    mypositions = [(0, -0.1, 0),(0.0, -0.0, -10.1)]
    scan_mesh(cmesh, mypositions, "file")

    # o3d.io.write_point_cloud(str(OUTPATH / 'crop.ply')
    # pcl = stl2pcl(crop)
    # o3d.io.write_triangle_mesh('crop.stl', crop)
    # show_pcl(pcl)
    # hide_point(pcl)
    #mesh2ply(INFIL,OUTPATH / 'Bridge1.ply')
    #meshsurface2ply(INFIL, OUTPATH / 'fil1.ply', OUTPATH / 'fil2.ply')
