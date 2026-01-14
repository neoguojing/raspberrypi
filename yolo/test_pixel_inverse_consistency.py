import numpy as np
import cv2
import math
from yolo.zen_seg import ZenohSegScan   # ← 改成你的文件名

def project_ground_point_to_pixel(node, X, Y):
    # 1. ground → base_link
    Pw = np.array([
        [X - node.camera_x_offset,
         Y,
         -node.camera_height]
    ], dtype=np.float32)

    # 2. base → optical (REP-103)
    Pw_opt = np.array([
        [-Pw[0,1],   # -Y
         -Pw[0,2],   # -Z
          Pw[0,0]]   # +X
    ], dtype=np.float32)

    # 3. undo pitch（相机坐标系）
    p = node.camera_pitch
    R = np.array([
        [ np.cos(-p), 0, -np.sin(-p)],
        [ 0,          1,  0         ],
        [ np.sin(-p), 0,  np.cos(-p)]
    ], dtype=np.float32)

    Pw_cam = (R @ Pw_opt.T).T

    # 4. 使用 OpenCV 正确投影（🔥关键）
    rvec = np.zeros((3,1), dtype=np.float32)
    tvec = np.zeros((3,1), dtype=np.float32)

    imgpts, _ = cv2.projectPoints(
        Pw_cam,
        rvec,
        tvec,
        node.K,
        node.dist_coeffs
    )

    u, v = imgpts[0,0]
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
