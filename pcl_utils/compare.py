import numpy as np
from matplotlib import pyplot, image
#import open3d as o3d
from pathlib import Path

_DEBUG=True

def compare_pcl(fpcl1, fpcl2):
    "compare 2 pointcloud and return a value for the error"
    if not fpcl1.exists() or not fpcl2.exists():
        print("File does not exist", fpcl1, fpcl2)
        raise Exception("Input not found")
    pcl1 = o3d.io.read_point_cloud(str(fpcl1))
    print(pcl1)
    pcl2 = o3d.io.read_point_cloud(str(fpcl2))
    if _DEBUG:
        print("Points in original", len(pcl1.points))
        print("Point in pointcloud", len(pcl2.points))
    dist = pcl1.compute_point_cloud_distance(pcl2)
    distance = np.asarray(dist)
    print("Error max", np.max(distance), " Min", np.min(distance))
    pclerror = np.sqrt(np.mean(distance ** 2))
    print("meansquare", pclerror)
    return pclerror



def show_diff(file1, file2):
    img1 = image.imread(file1)
    print("absimg1 max", np.max(img1), "min", np.min(img1))

    n1 = np.asarray(img1)   
    pyplot.imshow(n1)
    print(img1.shape)
    print("img1 max", np.max(n1), "min", np.min(n1))
    pyplot.show()
    
    img2 = image.imread(file2)
    n2 = np.asarray(img2)
    print("img2 max", np.max(n2), "min", np.min(n2))
    pyplot.imshow(n2)
    pyplot.show()
      
    n = n1 - n2
    #print (n)
    print("diff max", np.max(n), "min", np.min(n))
    print("mean error", np.mean(np.abs(n)))
    
    pyplot.imshow(n, cmap='gray')
    pyplot.show()   
    # pyplot.imshow(img2)
    # pyplot.show()
    
    return


    
    
    
    
# print("wrapped")
# file = "pictures/analytic/analog_im_wrap1.png"
# file2 = "pictures/inf/wrap.png"
# show_diff(file, file2)

print("K")
file = "pictures/analytic/kdata.png"
file2 = "pictures/inf/nnk.png"
show_diff(file, file2)

# print("unwrapped")
# file = "pictures/analytic/unwrap.png"
# file2 = "pictures/inf/unwrap.png"
# show_diff(file, file2)

# show_diff(file, file2)

# print ("Compare pointclouds")
# error = compare_pcl(Path("pictures/analytic/pointcl-depth.ply"), Path("pictures/inf/pcl.ply"))
# print("pcl compare error", error)
