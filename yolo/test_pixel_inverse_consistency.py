import numpy as np
import cv2
import math
from yolo.zen_seg import ZenohSegScan   # ← 改成你的文件名

def project_ground_point_to_pixel(node, X, Y):
    """
    数学原理：透视投影变换 (Perspective Projection)
    流程：World(Base) -> Camera_Relative -> Camera_Tilt_Adjusted -> Optical_Frame -> Pixel
    """

    # --- 1. 刚体变换：平移 (Translation) ---
    # 数学原理：将坐标原点从机器人重心(base_link)移至相机光心
    # P_rel = P_base - T_camera_in_base
    # 这里假设相机安装在 (x_offset, 0, height)
    dx = X - node.camera_x_offset
    dy = Y - 0.0
    dz = 0.0 - node.camera_height

    # --- 2. 刚体变换：旋转 (Rotation - Pitch) ---
    # 数学原理：绕 Y 轴（机器人左/右方向）旋转。
    # 旋转矩阵 R_y(p) = [[cos p, 0, -sin p], [0, 1, 0], [sin p, 0, cos p]]
    # 作用：处理相机的俯仰角，将坐标从水平相机位姿转到实际倾斜位姿
    # nx = dx*cos(p) - dz*sin(p)  (旋转后的前向距离)
    # nz = dx*sin(p) + dz*cos(p)  (旋转后的高度偏移)
    p = node.camera_pitch
    c, s = np.cos(p), np.sin(p)
    
    nx = dx * c - dz * s
    ny = dy
    nz = dx * s + dz * c

    # --- 3. 坐标系重映射 (Coordinate Mapping - REP 103) ---
    # 数学原理：机器人坐标系与相机光学坐标系的对齐
    # 机器人标准 (REP-103): X-前, Y-左, Z-上
    # 相机光学标准 (Optical): Z-前(光轴), X-右, Y-下
    # 映射关系：
    # Opt_X = -Base_Y (左转右)
    # Opt_Y = -Base_Z (上转下)
    # Opt_Z =  Base_X (前转前)
    p_opt = np.array([[-ny, -nz, nx]], dtype=np.float32)

    # --- 4. 相机内参投影与畸变处理 (Intrinsic Projection & Distortion) ---
    # 数学原理：针孔相机模型 (Pinhole Camera Model)
    # 1. 归一化投影：xn = Opt_X / Opt_Z, yn = Opt_Y / Opt_Z
    # 2. 畸变校正：应用 Brown-Conrady 模型处理多项式径向/切向畸变 (dist_coeffs)
    # 3. 仿射变换：u = fx * xn_distorted + cx, v = fy * yn_distorted + cy
    # cv2.projectPoints 自动完成了从 3D 空间到 2D 像素平面的非线性映射
    imgpts, _ = cv2.projectPoints(
        p_opt.reshape(1,1,3),
        np.zeros((3,1)), np.zeros((3,1)), # 旋转和平移已在前面手动处理完成
        node.K, node.dist_coeffs
    )
    return imgpts[0,0]


def project_ground_points_to_pixel_batch(node, points_2d):
    """
    批量透视投影变换：将地面坐标系下的多个 (X, Y) 点投影到图像像素坐标
    Args:
        points_2d: np.ndarray, shape [N, 2], 元素为 [[X1, Y1], [X2, Y2], ...]
    Returns:
        pixels: np.ndarray, shape [N, 2], 元素为 [[u1, v1], [u2, v2], ...]
    """

    points_2d = np.asanyarray(points_2d)
    if len(points_2d) == 0:
        return np.empty((0, 2))

    # --- 1. 刚体变换：平移 (Translation) ---
    # 假设所有点都在地面上，即 Z = 0
    n = len(points_2d)
    dx = points_2d[:, 0] - node.camera_x_offset
    dy = points_2d[:, 1] - 0.0
    dz = np.full(n, -node.camera_height)

    # --- 2. 刚体变换：旋转 (Rotation - Pitch) ---
    p = node.camera_pitch
    c, s = np.cos(p), np.sin(p)
    
    # 批量应用旋转矩阵
    nx = dx * c - dz * s
    ny = dy
    nz = dx * s + dz * c

    # --- 3. 坐标系重映射 (Coordinate Mapping) ---
    # 构造光学坐标系下的点集 p_opt: [N, 3]
    # Opt_X = -Base_Y, Opt_Y = -Base_Z, Opt_Z = Base_X
    p_opt = np.column_stack([-ny, -nz, nx]).astype(np.float32)

    # --- 4. 批量相机内参投影与畸变处理 ---
    # cv2.projectPoints 能够处理 [N, 3] 或 [N, 1, 3] 形状的数组
    imgpts, _ = cv2.projectPoints(
        p_opt.reshape(-1, 1, 3),
        np.zeros((3, 1)), np.zeros((3, 1)), # 旋转和平移已在前面处理，这里传 0
        node.K, node.dist_coeffs
    )

    # 结果 shape 从 [N, 1, 2] 展平为 [N, 2]
    return imgpts.reshape(-1, 2)


def test_inverse_consistency():
    node = ZenohSegScan(config_path='robot/config/imx219.json')

    test_points = [
        # --- 1. 中心线测试（验证 X轴投影与 Pitch） ---
        (0.4,  0.0),   # 极近点（测试 range_min 边缘）
        (1.0,  0.0),   # 标准近点
        (3.0,  0.0),   # 中距点
        (6.0,  0.0),   # 远距点（验证远距离下像素微偏带来的误差放大）

        # --- 2. 左右对称测试（验证 Y轴与横向偏移） ---
        (1.5,  0.5),   # 左侧
        (1.5, -0.5),   # 右侧
        (2.0,  1.2),   # 大角度左侧（可能接近图像边缘，测试畸变参数）
        (2.0, -1.2),   # 大角度右侧

        # --- 3. 边界与无效点（验证过滤逻辑） ---
        (-0.5, 0.0),   # 机器人后方（应该返回 None 或被 project 判定为 behind camera）
        (0.1,  0.0),   # 相机下方/中心偏移处（验证 camera_x_offset）
        (10.0, 0.0),   # 超出 range_max 的点
    ]


    print("\n=== Forward-Inverse Consistency Test ===")

    for X, Y in test_points:
        uv = project_ground_point_to_pixel(node, X, Y)
        if uv is None:
            print(f"[SKIP] ({X:.2f},{Y:.2f}) behind camera")
            continue

        res = node.pixel_to_base(*uv)
        if res is None:
            print(f"[FAIL] pixel_to_base returned None for {uv}")
            continue

        Xp, Yp = res
        err = math.hypot(X - Xp, Y - Yp)

        print(
            f"GT=({X:.2f},{Y:.2f}) → "
            f"uv={uv} → "
            f"({Xp:.3f},{Yp:.3f}) | "
            f"err={err*100:.2f} cm"
        )

    uvpoints = project_ground_points_to_pixel_batch(node,test_points)
    results = node.pixel_to_base_batch(uvpoints)

    for i, res in enumerate(results):
        X, Y = test_points[i]
        Xp, Yp = res
        err = math.hypot(X - Xp, Y - Yp)

        print(
            f"GT=({X:.2f},{Y:.2f}) → "
            f"uv={uv} → "
            f"({Xp:.3f},{Yp:.3f}) | "
            f"err={err*100:.2f} cm"
        )
if __name__ == "__main__":
    test_inverse_consistency()
