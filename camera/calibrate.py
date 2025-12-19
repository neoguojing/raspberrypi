import cv2
import time
import numpy as np
import os
import json
from pathlib import Path
from camera.camera import RpiCamera


# ============================================================
#                Camera Calibration Tool
# ============================================================
class CalibrationTool:
    """
    提供：
        - 采集棋盘格图像（支持从队列获取帧并跳帧）
        - 相机标定（K, dist）
        - 计算单应矩阵 H（世界平面 → 像素）
        - 保存/加载 JSON（方便其他模块使用）
    """

    def __init__(self,
                 camera: RpiCamera,
                 board_size=(8, 5),
                 square_size=0.024,
                 frame_skip=5):
        """
        参数说明:
            camera       : RpiCamera 实例，get_frame() 从队列拿取最新帧
            board_size   : 棋盘格内角点数量 (cols, rows)
            square_size  : 每个格子的真实世界尺寸（单位随意，一般 meters）
            frame_skip   : 跳过多少队列帧，避免旧帧导致模糊
        """

        self.camera = camera
        self.board_size = board_size
        self.square_size = square_size
        self.frame_skip = frame_skip

        self.K = None
        self.dist = None
        self.reproj_error = None
        self.H = None
        self.img_shape = None

    # ============================================================
    #                获取最新帧的“跳帧策略”
    # ============================================================
    def _get_latest_frame(self):
        """
        RpiCamera.get_frame() 是从 queue 中取帧。
        正确的“跳帧策略”应该是：
            - 连续调用 get_frame 丢弃旧帧
            - 最后一次 get_frame 的帧即最新帧

        好处：
            ✓ 不会积累延迟
            ✓ 不依赖摄像头 fps
            ✓ FrameQueue 堆积旧帧时不会使用错误帧
        """
        frame = None
        for _ in range(self.frame_skip):
            _, frame = self.camera.get_frame(rgb=False)  # get latest bgr frame
        return frame

    # ============================================================
    #                 采集校准图片
    # ============================================================
    def capture_calibration_images(
        self, count=50, delay=1.0, out_dir="calib_images", show=False
    ):
        """
        自动采集棋盘格图片，用于后续标定。

        参数:
            count   : 采集数量
            delay   : 两张图片之间的时间间隔（秒）
            out_dir : 保存目录
            show    : 是否弹出显示窗口查看实时画面
        """

        out = Path(out_dir)
        out.mkdir(exist_ok=True)

        saved = 0
        print(f"[Calib] 开始采集 {count} 张图片...")
        self.camera.start()

        while saved < count:
            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            if show:
                cv2.imshow("calib preview", frame)
                cv2.waitKey(1)

            save_path = out / f"calib_{saved:03d}.jpg"
            cv2.imwrite(str(save_path), frame)
            print(f"[Calib] 保存: {save_path}")
            saved += 1

            time.sleep(delay)

        if show:
            cv2.destroyAllWindows()

        print("[Calib] 采集完成")
        return True

    # ============================================================
    #                棋盘格相机标定 K, dist
    # ============================================================
    def calibrate_camera(
        self,
        images=None,
        image_dir="calib_images",
        visualize=True,
        show=False,
        vis_dir="output"
    ):
        """
        参数:
            images      : 明确传入图片路径数组
            image_dir   : 若 images=None，则自动读取该目录下所有 jpg/png
            visualize   : 是否保存角点可视化结果
            show        : 是否在屏幕上展示角点（树莓派建议 False）
            vis_dir     : 可视化结果保存目录，默认 "output"

        返回 dict:
            camera_matrix (3x3)
            dist_coeffs (1x5)
            reprojection_error
            image_size (w, h)
        """

        # -----------------------------
        # 自动读取目录内所有图像
        # -----------------------------
        if images is None:
            image_dir = Path(image_dir)
            images = sorted([
                str(p) for p in image_dir.glob("*.jpg")
            ] + [
                str(p) for p in image_dir.glob("*.png")
            ])

        if len(images) == 0:
            raise RuntimeError("未找到任何校准图片，请先执行 capture_calibration_images()")

        # 若开启可视化，准备目录
        if visualize:
            vis_path = Path(vis_dir)
            vis_path.mkdir(exist_ok=True, parents=True)

        # -----------------------------
        # 构建棋盘格 3D 物理坐标
        # -----------------------------
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        # (0..W-1, 0..H-1) 网格展开
        objp[:, :2] = np.indices(self.board_size).T.reshape(-1, 2)
        objp *= self.square_size  # 乘上实际每格长度（米/厘米单位由你决定）

        objpoints = []   # 3D 世界点
        imgpoints = []   # 2D 像素点
        img_shape = None # 保存标定图像大小 (w, h)

        # -----------------------------
        # 检测角点
        # -----------------------------
        for path in images:
            img = cv2.imread(path)
            if img is None:
                print(f"[Calib] 无法读取: {path}")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_shape = gray.shape[::-1]  # (width, height)

            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.board_size)

            if not ret:
                print(f"[Calib] ❌ 未找到角点: {path}")
                continue

            # 亚像素角点优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners_sub)

            print(f"[Calib] ✔ 角点 OK: {path}")

            # ------------------------------------
            # 角点可视化：绘制并保存图片
            # ------------------------------------
            if visualize:
                vis_img = img.copy()
                cv2.drawChessboardCorners(vis_img, self.board_size, corners_sub, ret)

                save_path = str(Path(vis_dir) / (Path(path).stem + "_corners.jpg"))
                cv2.imwrite(save_path, vis_img)

                if show:
                    cv2.imshow("Corners", vis_img)
                    cv2.waitKey(200)

        if show:
            cv2.destroyAllWindows()

        # -----------------------------
        # 确保至少有 3 张有效图
        # -----------------------------
        if len(objpoints) < 3:
            raise RuntimeError("有效角点图像不足（至少 3 张）")

        # -----------------------------
        # 标定计算 K, dist
        # -----------------------------
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, img_shape, None, None
        )

        # -----------------------------
        # 计算重投影误差
        # -----------------------------
        total_err = 0
        for i in range(len(objpoints)):
            proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
            err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
            total_err += err

        reproj_error = total_err / len(objpoints)

        print(f"[Calib] 标定完成 ✔\n重投影误差: {reproj_error:.4f}")
        self.K = K
        self.dist = dist
        self.reproj_error = reproj_error
        self.img_shape = img_shape
        return {
            "camera_matrix": K,
            "dist_coeffs": dist,
            "reprojection_error": float(reproj_error),
            "image_size": img_shape,
        }


    # ============================================================
    #                单应矩阵 (world->pixel)
    # ============================================================
    @staticmethod
    def compute_homography(self,world_pts, image_pts):
        """
        world_pts: Nx2 世界坐标点（平面坐标）
        image_pts: Nx2 像素坐标点
        """
        world_pts = np.array(world_pts, dtype=np.float32)
        image_pts = np.array(image_pts, dtype=np.float32)
        H, mask = cv2.findHomography(world_pts, image_pts, cv2.RANSAC)
        print(f"[Calib] 单应矩阵 H:\n{H}")
        self.H = H
        return H, mask

    # ============================================================
    #                保存所有标定结果为一个 JSON
    # ============================================================
    def save_calibration_json(self,path="./imx219.json"):
        """
        参数:
            path : 保存 JSON 路径
            K    : 3x3 相机矩阵
            dist : 畸变系数
            H    : 单应矩阵
        """
        data = {
            "camera_matrix": self.K.tolist(),
            "dist_coeffs": self.dist.flatten().tolist(),
            "homography": None if self.H is None else self.H.tolist(),
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=4)
        print(f"[Calib] JSON 写入: {path}")

    # ============================================================
    #                从 JSON 恢复 numpy 参数
    # ============================================================
    def load_calibration_json(self,path="./imx219.json"):
        with open(path, "r") as f:
            data = json.load(f)

        self.K = np.array(data["camera_matrix"], dtype=np.float32)
        self.dist = np.array(data["dist_coeffs"], dtype=np.float32)
        if data["homography"] is not None:
            self.H = np.array(data["homography"], dtype=np.float32)

        return data
    
if __name__ == "__main__":
    my_cam = RpiCamera()
    tool = CalibrationTool(camera=my_cam, frame_skip=5)

    # Step 1: 采集图像
    # tool.capture_calibration_images(count=40)

    # Step 2: 标定 K、dist
    calib = tool.calibrate_camera()

    # # Step 3: 计算单应矩阵
    # H, _ = tool.compute_homography(
    #     world_pts=[(0,0), (1,0), (1,1), (0,1)],
    #     image_pts=[(320,240), (640,240), (640,480), (320,480)]
    # )

    # Step 4: 保存所有参数到一个 JSON
    tool.save_calibration_json()

    # Step 5: 加载
    params = tool.load_calibration_json("calibration_output.json")
    K = params["camera_matrix"]
    dist = params["dist_coeffs"]
    H = params["homography"]

