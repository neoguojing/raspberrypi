import numpy as np
import cv2
import math
from yolo.zen_seg import ZenohSegScan   # ← 改成你的文件名

def project_ground_point_to_pixel(node, X, Y):
    # 1. ground → base_link
    Pw = np.array([
        X - node.camera_x_offset,
        Y,
        -node.camera_height
    ], dtype=np.float32)

    # 2. undo pitch（⚠️ 在 base_link 坐标系中）
    p = node.camera_pitch
    c, s = np.cos(-p), np.sin(-p)
    R_pitch = np.array([
        [ c, 0, -s],
        [ 0, 1,  0],
        [ s, 0,  c]
    ], dtype=np.float32)

    Pw_base = R_pitch @ Pw

    # 3. base_link → optical（REP-103）
    Pw_opt = np.array([
        -Pw_base[1],   # -Y
        -Pw_base[2],   # -Z
         Pw_base[0]    # +X
    ], dtype=np.float32)

    # 4. OpenCV 投影（带畸变）
    Pw_opt = Pw_opt.reshape(1, 1, 3)

    rvec = np.zeros((3,1), dtype=np.float32)
    tvec = np.zeros((3,1), dtype=np.float32)

    imgpts, _ = cv2.projectPoints(
        Pw_opt,
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
