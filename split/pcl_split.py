"Split pointcload for stitching test"
import colorsys
from math import pi, sin ,cos
import random
from pathlib import Path
import shutil
import open3d as o3d
import math

# pylint: disable=invalid-name

_DEBUG = True

def gen_color_list(number):
    "make a rainbow list"
    colorlist = []
    for i in range(number):
        hue = i/number
        #print(i, hue)
        #c = (hue, 1, 1)
        rgb = colorsys.hsv_to_rgb(hue,1,1)
        #print(rgb)
        colorlist.append(rgb)
    #print(colorlist)
    colorlist = [(1,0,0),(0,1,0),(0,0,1),(1,1,0),(1,0,1),(0,1,1)]
    return colorlist

def crop_pcl(pcl, crop):
    "Crop pcl"
    minc = crop[0]
    maxc = crop[1]
    print("box",minc,maxc)
    bbox = o3d.geometry.AxisAlignedBoundingBox(minc,maxc)
    box = pcl.crop(bbox)
    box.estimate_normals()
    camera = (0,0,0)
    radius = 1
    mesh = box.hidden_point_removal(camera, radius)

    mesh[0].compute_triangle_normals()
    #print(mesh)
    #print(mesh[1])
    #print(m)
    o3d.io.write_triangle_mesh("ud.stl", mesh[0])
    box.paint_uniform_color((1,0,0))
    box.colors[0] = (1,1,1)
    print("has colors", box.has_colors())
    print("point", box.points[0])
    print('color', box.colors[0])
    print('normals', box.normals[0])
    return box

def show_pcl(pcd):
    "show the pcl for debugging"
    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = True ) #width=200, height=200
    if not res:
        print("create window result", res)
    vis.add_geometry(pcd)
    #ctr = vis.get_view_control()
    #if ctr is None:
    #    print("pcl2jpg cant get view_control", vis)
    # fix position
    #obj_center =OBJ_CENTER
    #cam_position=CAM_POSITION
    #if _DEBUG:
    #    print('object center', obj_center, "cam position:", cam_position, "zoom", zoom)
    #ctr.set_zoom(zoom)
    # ctr.set_front(cam_position)
    # ctr.set_lookat(obj_center)
    # ctr.set_up([+10.0, 0, 0])
    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.point_color_option = o3d.visualization.PointColorOption.YCoordinate
    vis.run()

def down_sample(filename, fileout):
    "downsamle the current file to 1 mm"
    filename = "chess.ply"
    file = filename
    pcl =o3d.io.read_point_cloud(str(file))
    outfile = Path(fileout)
    #pcl2jpg(pcl, outfile)
    print("Number of points: ", len(pcl.points))
    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcl.voxel_down_sample(voxel_size=0.5)
    #pcl2jpg(downpcd, Path(folder) / "pic2.jpg")
    #outfile = Path(folder) / "pic2.ply"
    print("Number of downsampled points: ", len(downpcd.points))
    o3d.io.write_point_cloud(str(outfile), downpcd)

def rx_matrix(vx):
    "create the rotation matrix about x"
    v= vx/180*pi
    matrix = [(1,0,0), (0,cos(v),-sin(v)), (0,sin(v),cos(v))]
    return matrix

def ry_matrix(vy):
    "create the rotation matrix y"
    v= vy/180*pi
    matrix = [(cos(v),-sin(v),0), (sin(v), cos(v),0),(0,0,1)]
    return matrix

def rz_matrix(vz):
    "create the rotation matrix z "
    v= vz/180*pi
    matrix = [(cos(v),0,sin(v)), (0,1,0), (-sin(v),0, cos(v))]
    return matrix

def rotate_pcl(pcl, mat):
    "rotate according to matrix"
    respcl = pcl.rotate(mat)
    return respcl

def split_pcl(file, outpath, split_number=18):
    "split a pcl in a number files for stitching"
    overlap = 1.3
    colors = gen_color_list(int(split_number*1.6))
    outpath = Path(outpath)
    pcl = o3d.io.read_point_cloud(str(file))
    print("points", len(pcl.points))
    print("get_axis_aligned_bounding_box", pcl.get_axis_aligned_bounding_box())
    maxb = pcl.get_max_bound()
    minb = pcl.get_min_bound()
    print("get_max_bound", maxb)
    cubic = (maxb[0]-minb[0])*(maxb[1]-minb[1])*(maxb[2]-minb[2])
    prcube = cubic/split_number
    print("splitnumber", split_number)
    print("Cubic", cubic, "prcube", prcube)
    side = prcube ** (1/3)
    print(f"side {side} m")
    #splitsize = int(side)
    #splitsize = int((pcl.get_max_bound()[0] - pcl.get_min_bound()[0])/split_number)
    splitsize = (pcl.get_max_bound()[0] - pcl.get_min_bound()[0])/split_number
    #size = int(splitsize * overlap)
    size = splitsize * overlap
    print(f'splitsize {splitsize} size {size}')
    #xrange = range(int(pcl.get_min_bound()[0]),int(pcl.get_max_bound()[0]),splitsize)
    #yrange = range(int(pcl.get_min_bound()[1]),int(pcl.get_max_bound()[1]),splitsize)
    #zrange = range(int(pcl.get_min_bound()[2]),int(pcl.get_max_bound()[2]),splitsize)
    #print ("x-range", xrange)
    print("split-size", splitsize)

    # convert to mm

    xrange = range(-30, 30, 10)
    yrange = range(-30, 30, 10)
    zrange =range(10, 40, 10)
    sumpoint = 0
    number = 0
    size = 0.01
    for x in xrange:
        for y in yrange:
            for z in zrange:
                #print("xyz", x, y, z)
                minc = (x/1000,y/1000,z/1000)
                maxc = (x/1000+size,y/1000+size,z/1000+size)
                print("box",minc,maxc)
                box = o3d.geometry.AxisAlignedBoundingBox(minc,maxc)
                cube = pcl.crop(box)
                if not cube.has_colors():
                    cube.paint_uniform_color(colors[number % len(colors)])
                nopoints = len(cube.points)
                #colorarr = np.full(1, 44)
                if nopoints>0:
                    sumpoint += len(cube.points)
                    outfile = outpath / f"file{number}.ply"
                    o3d.io.write_point_cloud(str(outfile), cube)
                    number +=1
                    #o3d.visualization.draw_geometries([cube])
                    #print("file:", number, "points", len(cube.points))
                    #show_pcl(cube)
    print("Number files", number)
    print("Sumpoints", sumpoint)

def random_split(file, outpath, splitsize=10):
    "split a pcl in a number files for stitching and random rotate"
    xrange = range(-30,30,splitsize)
    yrange = range(-30,30,splitsize)
    zrange = range(-30,30,splitsize)
    size = int(splitsize * 1)
    outpath = Path(outpath)
    pcl = o3d.io.read_point_cloud(str(file))
    sumpoint = 0
    number = 0

    for x in xrange:
        for y in yrange:
            for z in zrange:
                minc = (x,y,z)
                maxc = (x+size,y+size,z+size)
                box = o3d.geometry.AxisAlignedBoundingBox(minc,maxc)
                cube = pcl.crop(box)
                nopoints = len(cube.points)
                if nopoints>0:
                    sumpoint += len(cube.points)
                    print ("Points in box", len(cube.points ))
                    randomv = random.randint(0,50)
                    resrot  = rotate_pcl(cube, rx_matrix(randomv))
                    outfile = outpath / f"file{number}.ply"
                    o3d.io.write_point_cloud(str(outfile), resrot)
                    number +=1

if __name__ == "__main__":
    FILE = Path('testdata/pcl/tooth/Bridge1.ply')
    outfolder = FILE.parent / 'Bridge1'
    outfolder.mkdir(exist_ok=True)
    pcl = o3d.io.read_point_cloud(str(FILE))
    opcl = crop_pcl(pcl,((-1, 0.010, 0),(-0.01,1,20)))
    o3d.io.write_point_cloud(str(outfolder / "crop.ply"), opcl)
    #split_pcl(FILE, outfolder, split_number=4*4*4)
    #outfolder = "out/"+f+'/random'
    #Path(outfolder).mkdir(parents=True, exist_ok=True)
    #random_split(infile, outfolder, splitsize=30)
 