"pointcload utils"
from pathlib import Path
import numpy as np
#from matplotlib import use, pyplot as plt
#from matplotlib import use

O3D = True
if O3D:
    import open3d as o3d

_DEBUG = False

PICTURE_SIZE = 1000
OBJ_CENTER = [0.0,0.0,22.0]
CAM_POSITION = [-0.0, -0.0, -0.0]
ZOOM = 1  # tand

def pcl2jpg(pcd, outfile, cam='s', zoom=ZOOM):
    obj_center = pcd.get_center()
    if _DEBUG:
        arr = np.asarray(pcd.points)
        amin = np.min(arr, axis=0)
        amax = np.max(arr, axis=0)
        print("Object limits min, max", amin, amax)
        print("Object center", obj_center)
    cam_position = obj_center.copy()
    #cam_position[2] = CAM_POSITION[2]   #lodret
    # camera position
    diff = 10
    if cam=='n':
        cam_position[0] += diff
    elif cam=='e':
        cam_position[1] -= diff
    elif cam=='w':
        cam_position[1] += diff
    elif cam=='s':
        cam_position[0] -= diff
    else:
        print("Error in pcl to jpg position")
        cam_position[0] += diff
    cam_position = CAM_POSITION
    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = _DEBUG, width=PICTURE_SIZE, height=PICTURE_SIZE)
    if not res:
        print("create window result", res)
    vis.add_geometry(pcd)
    ctr = vis.get_view_control()
    if ctr is None:
        print("pcl2jpg cant get view_control", vis)
    # fix position
    obj_center =OBJ_CENTER
    cam_position=CAM_POSITION
    if _DEBUG:
        print('object center', obj_center, "cam position:", cam_position, "zoom", zoom)
    ctr.set_zoom(zoom)
    # ctr.set_front(cam_position)
    # #ctr.set_lookat(obj_center)
    # ctr.set_lookat([-30,-20,-10])
    # ctr.set_up([+10.0, 0, 0])
    opt = vis.get_render_option()
    #opt.background_color = ([0.40,0.40,0.40])
    opt.point_size = 1.0
    opt.point_color_option = o3d.visualization.PointColorOption.YCoordinate
    #opt.point_color_option = o3d.visualization.PointColorOption.Normal
    if _DEBUG:
        vis.run()
    # if _DEBUG:
    #     img = vis.capture_screen_float_buffer(True)
    #     plt.imshow(np.asarray(img))
    vis.capture_screen_image(str(outfile), do_render=True)

def ply2jpg(infile, outfile, cam='s', zoom=ZOOM):
    pcd = o3d.io.read_point_cloud(str(infile))
    print("number points", len(pcd.points))
    pcl2jpg(pcd, outfile, cam=cam, zoom=zoom)


if __name__== "__main__":
    print("starter")
    for dir in ['LJ3','t9UJscan']:
        for filnr in range(18):
            file = 'out/' + dir + '/norm/file'+ str(filnr) + '.ply'
            ofile = 'out/' + dir +'/norm/file' + str(filnr) + ".jpg"
            if Path(file).exists():
                print(file,ofile)
                ply2jpg(file,ofile)
                file = 'out/' + dir + '/random/file'+ str(filnr) + '.ply'
                ofile = 'out/' + dir +'/random/file' + str(filnr) + ".jpg"
                print(file,ofile)
                ply2jpg(file,ofile)
