from pathlib import Path
import math
import open3d as o3d
import numpy as np

#
# laver streger i rummet
#

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

def normalyze_vector(vec):
    lgt = math.sqrt((vec[0]**2) + (vec[1]**2) + (vec[2]**2))
    print("lgt", lgt)
    return ([vec[0]/lgt, vec[1]/lgt, vec[2]/lgt])

def ortogonal(vec):
    x=vec[0]
    y=vec[1]
    z = 1/vec[2]*(-vec[0]*x-vec[1]*y)
    return([x, y, z])




INFIL = Path('testdata/mm/fortand/fortand.cc.stl')
INFIL = Path('tmp/trans/file01.ply')



def gen_line(p1,p2, color=[1,0,0]):
    points = [
        p1, p2,
    ]
    print(points)
    lines = [
        [0, 1],
    ]
    colors = [color for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    #o3d.visualization.draw_geometries([line_set,], zoom=0.8)
    return line_set

# view point
x=-6
y=0
z=9.39
lineset = gen_line([x,y,z],[0,0,0])

view = normalyze_vector([x,y,z])
print("view", view)
vec2 = ortogonal(view)
print("vec2", vec2)
viewa = gen_line([x,y,z],[x+view[0], y+view[1], z+view[2]], (1,1,0))
vec2a = gen_line([x,y,z],[x+vec2[0], y+vec2[1], z+vec2[2]], (1,0,0))

#(-b, a, 0), or (-c, 0, a) or (0, -c, b).


a1=[-view[1], view[0], 0]
a2=[-view[2], 0, view[0]]
a3=[0, -view[2], view[1]]
veca1 = gen_line([x,y,z],[x+a1[0], y+a1[1], z+a1[2]], (0,1,0))
veca2 = gen_line([x,y,z],[x+a2[0], y+a2[1], z+a2[2]], (0,1,1))
veca3 = gen_line([x,y,z],[x+a3[0], y+a3[1], z+a3[2]], (0,0,1))
mymesh = o3d.io.read_point_cloud(str(INFIL))
show_geo([mymesh,  viewa, vec2a], name="test1")
show_geo([mymesh,  veca1, veca2, veca3], name="test2")
show_geo([mymesh,  viewa, vec2a, veca1, veca2, veca3], name="test3")


TM = [
    [],
    [],
    [],
    [-6, 0, 10]

]