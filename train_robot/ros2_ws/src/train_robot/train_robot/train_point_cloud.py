import numpy as np
import cv2


def _import_open3d():
    try:
        import open3d as o3d
    except ImportError as exc:
        raise ImportError(
            "open3d is installed with incompatible dependencies. "
            "Point cloud visualization requires a working open3d/pandas/numpy setup."
        ) from exc
    return o3d



def dep2pcd(
        depth_image,
        rgb_image,
        focal_length=2.97,
        h_aperture=5.539,
        v_aperture=3.092,
        clipping_near=0.1,
        clipping_far=5.0,
        resolution=(1280, 720)
):
    """
    将深度图转换为点云数据（基于相机参数界面配置）
    
    参数:
        depth_image : numpy.ndarray
            输入的深度图（单位：米），形状为(H, W)
        focal_length : float
            焦距（毫米），默认1.81（来自界面参数）
        h_aperture : float
            水平光圈（毫米），默认3.88
        v_aperture : float
            垂直光圈（毫米），默认2.44
        clipping_near : float
            近裁剪面（米），默认0.1
        clipping_far : float
            远裁剪面（米），默认6.0
        resolution : tuple
            图像分辨率（宽，高），默认(1280, 720)
    返回:
        open3d.geometry.PointCloud
            转换后的点云对象
    """
    o3d = _import_open3d()

    # 参数验证
    assert len(depth_image.shape) == 2, "深度图必须是单通道2D数组"
    height, width = depth_image.shape
    if resolution is not None:
        assert (width, height) == resolution, f"深度图分辨率与预期不符，应为{resolution}"

    # 1. 计算相机内参（从毫米转换为像素单位）
    # 公式: fx = (focal_length * image_width) / sensor_width
    fx = (focal_length * width) / h_aperture
    fy = (focal_length * height) / v_aperture
    cx = width / 2.0  # 假设光心在图像中心
    cy = height / 2.0

    # 2. 创建像素坐标网格
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u = u.astype(float)
    v = v.astype(float)
    depth_clean = np.nan_to_num(depth_image, nan=0.0)

    # 3. 转换为相机坐标系（单位：米）
    # 注意：这里使用原始深度值（不应用裁剪范围，保留后续处理）
    z = depth_clean.astype(float)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # 4. 过滤无效点（基于裁剪范围和无效深度）
    valid_mask = (z > clipping_near) & (z < clipping_far) & (z != 0)
    points = np.stack((x[valid_mask], -y[valid_mask], -z[valid_mask]), axis=-1)

    # 5. 创建Open3D点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if rgb_image is not None:
        # rgb shape should be H,W,3
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        colors = rgb_image.reshape(-1, 3)[(v*width + u)[valid_mask].astype(int)]
        # normalize 0-255 -> 0-1
        colors = colors.astype(np.float32) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    pcd_small = sample_point_cloud(pcd, 4096, o3d=o3d)

    o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([pcd_small])

    return pcd

def sample_point_cloud(pcd, num_points=40000, o3d=None):
    if o3d is None:
        o3d = _import_open3d()

    # 如果点太少不降采样
    if len(pcd.points) <= num_points:
        return pcd
    
    pcd = pcd.uniform_down_sample(int(len(pcd.points) / num_points * 2))
    # FPS
    import numpy as np
    pts = np.asarray(pcd.points)
    
    # farthest point sampling
    def fps(points, k):
        N = points.shape[0]
        centroids = np.zeros(k, dtype=np.int64)
        distance = np.ones(N) * 1e10
        farthest = np.random.randint(0, N)
        for i in range(k):
            centroids[i] = farthest
            centroid = points[farthest]
            dist = np.sum((points - centroid) ** 2, axis=1)
            mask = dist < distance
            distance[mask] = dist[mask]
            farthest = np.argmax(distance)
        return centroids

    idx = fps(pts, num_points)
    sampled_pts = pts[idx]
    
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(sampled_pts)
    
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)[idx]
        new_pcd.colors = o3d.utility.Vector3dVector(colors)

    return new_pcd







def train_visualize_pointcloud():
    pass
