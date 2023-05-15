"Div stl utils"
from pathlib import Path
import open3d as o3d

_DEBUG = True
_SHOW = False

_ZOOM=1
_FRONT=[200.0, 200.0, 20.0]
_LOOKAT=[0.0, 0.0, 0.0]
_UP=[0.0,0.0,10.0]

# pylint: disable=invalid-name

TEST_DATA = Path(__file__).parent.parent / 'testdata/stl'

def show_stl(path, name=None, coord=False):
    "Show the stl file"
    if not path.exists():
        print("File not found", path)
        return
    mesh = o3d.io.read_triangle_mesh(str(path))
    meshlist = [mesh]
    if name is None:
        name = str(path)
    #lookat = mesh.get_center()
    if coord:
        coord_size = mesh.get_max_bound()[2] - mesh.get_min_bound()[2]
        coord_size = 0.01
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=coord_size)
        meshlist.append(coord)
    mesh.paint_uniform_color([0.8,0.8,0.8])
    mesh.compute_triangle_normals()
    o3d.visualization.draw_geometries(meshlist, window_name=name, width=800, height=600,
                                zoom=_ZOOM,
                                front=_FRONT,
                                lookat=_LOOKAT,
                                up=_UP,
                                point_show_normal=False,
                                mesh_show_back_face=True,
                                mesh_show_wireframe=False)


def stl2jpg(path):
    "create jpg picture of stl"
    PICTURE_SIZE = 1000
    #OBJ_CENTER = [0.0,0.0,22.0]
    CAM_POSITION = [0.0, 30.0, 0.0]
    ZOOM = 0.9  # tand
    zoom = ZOOM
    if not path.exists():
        print("File not found", path)
        return
    outfile = path.with_suffix('.jpg')
    print(outfile)
    mesh = o3d.io.read_triangle_mesh(str(path))
    obj_center = mesh.get_center()
     # camera position
    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = _DEBUG, width=PICTURE_SIZE, height=PICTURE_SIZE)
    if not res:
        print("create window result", res)
    vis.add_geometry(mesh)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20, origin=[0, 0, 0])
    vis.add_geometry(mesh_frame)
    ctr = vis.get_view_control()
    if ctr is None:
        print("pcl2jpg cant get view_control", vis)
    # fix position
    cam_position=CAM_POSITION
    if _DEBUG:
        print('object center', obj_center, "cam position:", cam_position, "zoom", zoom)
    ctr.set_zoom(zoom)
    ctr.set_front(cam_position)
    ctr.set_lookat(obj_center)
    ctr.set_up([+10.0, 0, 0])
    opt = vis.get_render_option()
    #opt.point_size = 2.0
    opt.mesh_show_wireframe = True
    opt.mesh_show_back_face = True
    #opt.point_color_option.Color = 1
    if _DEBUG:
        vis.run()
    vis.capture_screen_image(str(outfile), do_render=True)

if __name__ == "__main__":
    test_data = Path(TEST_DATA) / "stl"
    for file in test_data.glob("*.stl"):
        print(file)

