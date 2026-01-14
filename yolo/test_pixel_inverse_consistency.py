import numpy as np
import cv2
import math
from yolo.zen_seg import ZenohSegScan   # ← 改成你的文件名

def project_ground_point_to_pixel(node, X, Y):
    # 1. 世界坐标系下的地面点（Z=0）
    Pw = np.array([[X, Y, 0.0]], dtype=np.float32)

    # 2. 相机在 base_link 中的位置
    C = np.array([
        node.camera_x_offset,
        0.0,
        node.camera_height
    ], dtype=np.float32)

    # 3. 世界 → 相机(base_link)坐标
    Pw_base = Pw - C

    # 4. 相机俯仰（base_link 中，绕 Y 轴）
    p = node.camera_pitch
    c, s = np.cos(-p), np.sin(-p)
    R_pitch = np.array([
        [ c, 0, -s],
        [ 0, 1,  0],
        [ s, 0,  c]
    ], dtype=np.float32)

    Pw_base = (R_pitch @ Pw_base.T).T

    # 5. base_link → optical（REP-103）
    Pw_opt = np.zeros_like(Pw_base)
    Pw_opt[:,0] = -Pw_base[:,1]   # X_opt = -Y_base
    Pw_opt[:,1] = -Pw_base[:,2]   # Y_opt = -Z_base
    Pw_opt[:,2] =  Pw_base[:,0]   # Z_opt =  X_base

    # 6. OpenCV 投影（带畸变）
    Pw_opt = Pw_opt.reshape(1, 1, 3)

    imgpts, _ = cv2.projectPoints(
        Pw_opt,
        rvec=np.zeros((3,1), dtype=np.float32),
        tvec=np.zeros((3,1), dtype=np.float32),
        cameraMatrix=node.K,
        distCoeffs=node.dist_coeffs
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
