import numpy as np
import cv2
import math
from yolo.zen_seg import ZenohSegScan   # ← 改成你的文件名

def project_ground_point_to_pixel(node, X, Y):
    # 1. 平移：相对于相机安装位置的矢量
    dx = X - node.camera_x_offset
    dy = Y - 0.0
    dz = 0.0 - node.camera_height

    # 2. 旋转：从 base_link 旋转到相机水平坐标系
    # 这里的旋转矩阵必须是上面 pixel_to_base 中 R 矩阵的逆（即转置）
    p = node.camera_pitch
    c, s = np.cos(p), np.sin(p)
    
    # R_y(p): [c, 0, -s; 0, 1, 0; s, 0, c]
    nx = dx * c - dz * s
    ny = dy
    nz = dx * s + dz * c

    # 3. 转换到 Optical 坐标系 (REP-103)
    # Base: X-前, Y-左, Z-上 -> Optical: Z-前, X-右, Y-下
    p_opt = np.array([[-ny, -nz, nx]], dtype=np.float32)

    # 4. 使用 cv2.projectPoints 投影
    imgpts, _ = cv2.projectPoints(
        p_opt.reshape(1,1,3),
        np.zeros((3,1)), np.zeros((3,1)), # 不再重复旋转平移
        node.K, node.dist_coeffs
    )
    return imgpts[0,0]




def test_inverse_consistency():
    node = ZenohSegScan(config_path='robot/config/imx219.json')

    test_points = [
        (0.5,  0.0),
        (1.0,  0.2),
        (2.0,  0.3),
        (3.0, -0.3),
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


if __name__ == "__main__":
    test_inverse_consistency()
