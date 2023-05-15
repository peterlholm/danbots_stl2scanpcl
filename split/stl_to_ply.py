"Convert stil file to pointcloud ply"
from pathlib import Path
import open3d as o3d

def mesh2ply(infile, outfile):
    "convert mesh corners to ply"
    mesh = o3d.io.read_triangle_mesh(str(infile))
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.colors = mesh.vertex_colors
    #pcd.normals = mesh.vertex_normals
    #print(mesh)
    print(pcd)
    o3d.io.write_point_cloud(str(outfile), pcd)

def meshsurface2ply(infile, outfile, outfile2):
    "convert mesh surfaces to ply"
    mesh = o3d.io.read_triangle_mesh(str(infile))
    no_points = 1000
    pcl1 = mesh.sample_points_poisson_disk(number_of_points=no_points)
    pcl2 = mesh.sample_points_uniformly(number_of_points=no_points)
    o3d.io.write_point_cloud(outfile, pcl1)
    o3d.io.write_point_cloud(outfile2, pcl2)
    return

if __name__=="__main__":
    INFIL = Path('testdata/stl/tooth/Bridge1.stl')
    OUTPATH = Path('testdata/pcl/tooth')
    print(f"starting converting {INFIL} to {OUTPATH}")
    mesh2ply(INFIL,OUTPATH / 'Bridge1.ply')
    #meshsurface2ply(INFIL, OUTPATH / 'fil1.ply', OUTPATH / 'fil2.ply')
