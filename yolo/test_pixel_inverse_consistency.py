import numpy as np
import cv2
import math
from yolo.zen_seg import ZenohSegScan   # ← 改成你的文件名

def project_ground_point_to_pixel(node, X, Y):
    """
    正向投影：
    地面点 (X,Y,0) → 像素 (u,v)
    与 pixel_to_base 完全对偶
    """

    # 1. 地面点 → base_link
    Pw = np.array([
        X - node.camera_x_offset,
        Y, 
        -node.camera_height    # ← 🔥 关键：相机在地面之上
    ])

    # 2. base_link → camera optical
    # 逆 REP-103:
    # Opt_X = -Base_Y
    # Opt_Y = -Base_Z
    # Opt_Z =  Base_X
    P_opt = np.array([
        -Pw[1],
        -Pw[2],
         Pw[0]
    ])

    # 3. undo pitch (逆旋转)
    p = node.camera_pitch
    c, s = math.cos(-p), math.sin(-p)
    R = np.array([
        [ c, 0, -s],
        [ 0, 1,  0],
        [ s, 0,  c]
    ])
    P_cam = R @ P_opt

    # 4. 投影到归一化像平面
    if P_cam[2] <= 0:
        return None

    xn = P_cam[0] / P_cam[2]
    yn = P_cam[1] / P_cam[2]

    # 5. 像素化
    fx, fy = node.K[0,0], node.K[1,1]
    cx, cy = node.K[0,2], node.K[1,2]

    u = fx * xn + cx
    v = fy * yn + cy
    return float(u), float(v)


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
