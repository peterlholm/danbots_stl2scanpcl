"test module"
#from copy import deepcopy
from pathlib import Path
#from shutil import rmtree
import open3d as o3d
#import numpy as np

# scanning resolution
# Omnicam: 79 True Definition: 54 Trios: 41 iTero: 34 point pr mm2
# point pr mm2 default to a point distance 1/16 = 0.062 mm
POINT_PR_MM2  = 256  # = 160 * 160 / 100
PAINT_COLOR = True

DEBUG = False




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

INFIL = Path('testdata/mm/fortand/fortand.cc.stl')
OUTPATH = Path(__file__).parent.parent / "tmp"
mymesh = o3d.io.read_triangle_mesh(str(INFIL))

orgpcl = stl2pcl(mymesh, number=10000)
orgpcl.paint_uniform_color([0,1,0])
axi = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=(0,0,0))
o3d.visualization.draw([orgpcl, axi])

o3d.io.write_point_cloud("tmp/test.ply", orgpcl, write_ascii=True)
