import numpy as np
import open3d as o3d


def depth2PointCloud(depth, rgb, depth_scale, clip_distance_max):
    """
    使用 RealSense depth frame + color frame 直接生成相機座標點雲 (x,y,z,r,g,b)。
    clip_distance_max: 最大距離 (公尺)，負值則不做截斷，僅丟棄 depth=0。
    """
    intr = depth.profile.as_video_stream_profile().intrinsics
    depth_m = np.asanyarray(depth.get_data()) * depth_scale
    color_np = np.asanyarray(rgb.get_data())

    rows, cols = depth_m.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)

    if clip_distance_max > 0:
        valid = (depth_m > 0) & (depth_m < clip_distance_max)
    else:
        valid = (depth_m > 0)
    valid = np.ravel(valid)

    z = depth_m
    x = z * (c - intr.ppx) / intr.fx
    y = z * (r - intr.ppy) / intr.fy

    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]

    r_ch = np.ravel(color_np[:, :, 0])[valid]
    g_ch = np.ravel(color_np[:, :, 1])[valid]
    b_ch = np.ravel(color_np[:, :, 2])[valid]

    pointsxyzrgb = np.dstack((x, y, z, b_ch, g_ch, r_ch)).reshape(-1, 6)
    return pointsxyzrgb


def get_intrinsic_matrix(frame, imwidth, imheight):
    intr = frame.profile.as_video_stream_profile().intrinsics
    return o3d.camera.PinholeCameraIntrinsic(
        imwidth, imheight, intr.fx, intr.fy, intr.ppx, intr.ppy
    )


def create_point_cloud_file2(vertices, filename):
    ply_header = """ply
	format ascii 1.0
	element vertex %(vert_num)d
	property float x
	property float y
	property float z
	property uchar red
	property uchar green
	property uchar blue
	end_header
	"""
    with open(filename, "w") as f:
        f.write(ply_header % dict(vert_num=len(vertices)))
        np.savetxt(f, vertices, "%f %f %f %d %d %d")


def createPointCloudO3D(color_frame, depth_frame):
    """
    用 Open3D 建立彩色點雲並翻正 (Y/Z 反轉)，保持彩色，不轉灰階。
    """
    color_np = np.asanyarray(color_frame.get_data())
    depth_np = np.asanyarray(depth_frame.get_data())
    h, w, _ = color_np.shape
    color_img = o3d.geometry.Image(color_np)
    depth_img = o3d.geometry.Image(depth_np)
    intrinsic = get_intrinsic_matrix(color_frame, w, h)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img, depth_img, convert_rgb_to_intensity=False
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # 翻正：Y/Z 取反
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])
    return pcd


def loadPointCloud(path="mame.ply"):
    pcd = o3d.io.read_point_cloud(path)
    o3d.visualization.draw_geometries([pcd])
    return pcd


def write_point_cloud(ply_filename, points):
    points = points.tolist()
    with open(ply_filename, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex %d\n" % len(points))
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for p in points:
            f.write("%f %f %f %d %d %d\n" % (p[0], p[1], p[2], p[3], p[4], p[5]))
