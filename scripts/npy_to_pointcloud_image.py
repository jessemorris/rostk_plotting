import numpy as np 
import matplotlib.pyplot as plt
import open3d as o3d 

if __name__=="__main__":
    
    file_name = "/home/jesse/Code/src/ros/src/rostk_plotting/records/08:04:2021-19:28:27/1.npy"

    points = np.load(file_name)

    print(points.shape)

    # points = points.flatten()
    points[~np.isfinite(points)] = 0
    print(points[:10])

    # points = np.reshape(points, (307200, 3))
    print(points.shape)

    # x = points[:, 0]
    # y = points[:, 1]
    # z = points[:, 2]
    # print(points.shape)

    # v = pptk.viewer(points)
    # v.attributes(points)
    # v.set(point_size = 0.01)
    print((points.dtype))
    print(points[10])

    pcd = o3d.geometry.PointCloud()
    #
    pcd.points = o3d.utility.Vector3dVector(points)
    # pcd.paint_uniform_color([1, 0.706, 0])
    # pcd = pcd.voxel_down_sample(voxel_size=100)
    # pcd.colors = o3d.utility.Vector3dVector(colors/65535)
    # pcd.normals = o3d.utility.Vector3dVector(normals)
    # print(pcd)
    o3d.visualization.draw_geometries([pcd])

    # import matplotlib.pyplot as plt
    # from mpl_toolkits.mplot3d import proj3d

    # fig = plt.figure(figsize=(8, 8))
    # ax = fig.add_subplot(111, projection='3d')

    # ax.scatter(x, y, z)
    # plt.show()