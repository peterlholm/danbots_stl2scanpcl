"pointcload utils"
from pathlib import Path
from matplotlib.patches import Patch
import numpy as np
from matplotlib import use, pyplot as plt
#from matplotlib import use

O3D = True
if O3D:
    import open3d as o3d
    
_DEBUG = False

def mirror_pcl(infile, outfile):
    "Mirror pcl about X axis"
    pcd = o3d.io.read_point_cloud(str(infile))
    arr = np.asarray(pcd.points)
    #print('xyz_load', arr)
    for pkt in arr:
        pkt[0] = -pkt[0]
        #print(pkt)
    #print('xyz_load', arr)
    opcd = o3d.geometry.PointCloud()
    opcd.points = o3d.utility.Vector3dVector(arr)
    o3d.io.write_point_cloud(str(outfile), opcd)

def filter_area(inpcd):
    "remove noise in rand"
    out = []
    for p in inpcd.points:
        if p[0]>-3:
            out.append(p)
    outpcd = o3d.geometry.PointCloud()
    outpcd.points = o3d.utility.Vector3dVector(out)
    return outpcd

def filter_f_area(infile, outfile):
    if not Path(infile).exists():
        print("filter_pcl: infile does not exist", infile)
        return False
    pcd = o3d.io.read_point_cloud(str(infile))
    opcd = filter_area(pcd)
    o3d.io.write_point_cloud(str(outfile), opcd)



def filter_pcl(infile, outfile, procent=0.2):
    "filter outer procent part of pcl"
    if not Path(infile).exists():
        print("filter_pcl: infile does not exist", infile)
        return False
    pcd = o3d.io.read_point_cloud(str(infile))
    #print("pcd", pcd)
    if pcd is None:
        print("no point cloud")
    arr = np.asarray(pcd.points)
    if _DEBUG:
        print("Filter rand", arr.min(axis=0), arr.max(axis=0))
    amin = arr.min(axis=0)
    amax = arr.max(axis=0)
    #xmin = amin[0] + procent*(amax[0]-amin[0])
    #xmax = amax[0] - procent*(amax[0]-amin[0])
    #ymin = amin[1] + procent*(amax[1]-amin[1])
    #ymax = amax[1] - procent*(amax[1]-amin[1])
    #print (xmin, xmax, ymin, ymax)
    #print (arr.shape)
    # X Axis
    #points = np.asarray(pcd_clean.points)
    mask_x_1 = arr[:,0] > (amin[0] + procent*(amax[0]-amin[0]))
    mask_x_2 = arr[:,0] < (amax[0] - procent*(amax[0]-amin[0]))
    #print(mask_x_1)
    #print(mask_x_2)
    # Y Axis
    mask_y_1 = arr[:,1] > (amin[1] + procent*(amax[1]-amin[1]))
    mask_y_2 = arr[:,1] < (amax[1] - procent*(amax[1]-amin[1]))
    # Z Axis
    # mask_z_1 = points[:,2] < 0.3 # Closer to floor
    # mask_z_2 = points[:,2] > -0.1 # Clooser to ceiling
    mask_x = np.logical_and(mask_x_1, mask_x_2) # Along table's wide
    mask_y = np.logical_and(mask_y_1, mask_y_2) # Along table's longitude
    #mask_z = np.logical_and(mask_z_1, mask_z_2) # Along table's height
    #mask = np.logical_and(mask_x, mask_y, mask_z)
    mask = np.logical_and(mask_x, mask_y)
    #print(mask)
    #pcd_clean = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(arr[mask])
    o3d.io.write_point_cloud(str(outfile), pcd)
    return True


PICTURE_SIZE = 1000
OBJ_CENTER = [0.0,0.0,22.0]
CAM_POSITION = [-10.0, -0.0, -25.0]
#ZOOM = 0.5  # skak
ZOOM = 0.4  # tand

def pcl2jpg(pcd, outfile, cam='s', zoom=ZOOM):
    obj_center = pcd.get_center()
    if _DEBUG:
        arr = np.asarray(pcd.points)
        #print ("shape", arr.shape)
        amin = np.min(arr, axis=0)
        amax = np.max(arr, axis=0)
        print("Object limits min, max, center", amin, amax, obj_center)
    cam_position = obj_center.copy()
    cam_position[2] = CAM_POSITION[2]   #lodret
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
    ctr.set_front(cam_position)
    ctr.set_lookat(obj_center)
    ctr.set_up([+10.0, 0, 0])
    opt = vis.get_render_option()
    opt.point_size = 2.0
    #opt.point_color_option.Color = 1
    if _DEBUG:
        vis.run()
    # if _DEBUG:
    #     img = vis.capture_screen_float_buffer(True)
    #     plt.imshow(np.asarray(img))
    vis.capture_screen_image(str(outfile), do_render=True)

def render_image(pcd, outfile):
    arr = np.asarray(pcd.points)
    #print(arr)
    axe = plt.axes(projection='3d')
    axe.scatter(arr[:,0], arr[:,1], arr[:,2], c = "#222222", s=0.1)
    plt.savefig(outfile)
    #plt.show()

def ply2jpg(infile, outfile, cam='s', zoom=ZOOM):
    pcd = o3d.io.read_point_cloud(str(infile))
    pcl2jpg(pcd, outfile, cam=cam, zoom=zoom)

#ply2jpg(Path(__file__+'/../../testdata/render0/pointcl-nndepth.ply'), 'ud.jpg')

# maybe not used
def pcl2png(infilename, outfilename):
    "write file with pointcloud"
    use('Agg')
    pcd = o3d.io.read_point_cloud(str(infilename))
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible = False)
    vis.add_geometry(pcd)
    #is.get_render_option().load_from_json("Imaging/display/RenderOption.json")
    ctr = vis.get_view_control()
    ctr.set_zoom(0.45)
    ctr.set_front([-0.084777469999256949, -0.30273583588141922, -0.94929647331784794])
    ctr.set_lookat([0.0,0.0,26.0])
    ctr.set_up([+10.20694, 0, 00])
    #vis.run()
    img = vis.capture_screen_float_buffer(True)
    plt.imshow(np.asarray(img))
    vis.capture_screen_image(str(outfilename), do_render=True)

def mask_pcl(pcl_filename, mask_filename, out_filename):
    "mask pointcloud from maskfile"
    print("Masking pcl", pcl_filename, mask_filename, out_filename)
    pcd = o3d.io.read_point_cloud(str(pcl_filename))
    arr = np.asarray(pcd.points)
    _mask = np.load(mask_filename)

    #print('xyz_load', arr)
    for pkt in arr:
        print(pkt)
        #pkt[0] = -pkt[0]
    #print('xyz_load', arr)
    opcd = o3d.geometry.PointCloud()
    opcd.points = o3d.utility.Vector3dVector(arr)
    o3d.io.write_point_cloud(str(out_filename), opcd)
