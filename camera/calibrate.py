import cv2
import time
import numpy as np
import os
import yaml
from camera.camera import RpiCamera

# ---------------- IMX219 / generic camera calibration helper ----------------
class CalibrationTool:
    def __init__(self, camera: RpiCamera, board_size=(9,6), square_size=0.024):
        """
        board_size: inner corners per chessboard row & column e.g. (9,6)
        square_size: real-world square size in meters (or any unit)
        """
        self.camera = camera
        self.board_size = board_size
        self.square_size = square_size
        self.collected_images = []

    def capture_calibration_images(self, count=20, delay=1.0, show=False, out_dir="calib_images"):
        os.makedirs(out_dir, exist_ok=True)
        captured = 0
        print(f"[Calib] Capturing {count} images, delay {delay}s")
        while captured < count:
            img = self.camera.get_frame(rgb=False)  # get BGR
            if img is None:
                time.sleep(0.1)
                continue
            # optionally show
            if show:
                cv2.imshow("calib preview", img)
                cv2.waitKey(1)
            filename = os.path.join(out_dir, f"calib_{captured:03d}.jpg")
            cv2.imwrite(filename, img)
            self.collected_images.append(filename)
            captured += 1
            print(f"[Calib] Saved {filename}")
            time.sleep(delay)
        if show:
            cv2.destroyAllWindows()
        return self.collected_images

    def calibrate_from_images(self, images=None, save_path="camera_calib.npz", verbose=True):
        if images is None:
            images = self.collected_images
        objp = np.zeros((self.board_size[1]*self.board_size[0], 3), np.float32)
        objp[:, :2] = np.indices((self.board_size[0], self.board_size[1])).T.reshape(-1,2)
        objp *= self.square_size

        objpoints = []
        imgpoints = []
        img_shape = None
        for fname in images:
            img = cv2.imread(fname)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_shape = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
            if ret:
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), term)
                objpoints.append(objp)
                imgpoints.append(corners2)
                if verbose:
                    print(f"[Calib] Found corners in {fname}")
            else:
                if verbose:
                    print(f"[Calib] No corners in {fname}")

        if len(objpoints) < 3:
            raise RuntimeError("Not enough valid calibration images with detected corners.")

        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)
        # reprojection error
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        mean_error /= len(objpoints)

        np.savez(save_path, camera_matrix=K, dist_coeffs=dist, rvecs=rvecs, tvecs=tvecs, reprojection_error=mean_error)
        if verbose:
            print(f"[Calib] Saved calibration to {save_path}, reprojection_error={mean_error:.6f}")
        return {"camera_matrix": K, "dist_coeffs": dist, "rvecs": rvecs, "tvecs": tvecs, "reproj_err": mean_error}

    @staticmethod
    def save_yaml(save_path, cam_data):
        # cam_data: dict with 'camera_matrix' and 'dist_coeffs'
        data = {
            "camera_matrix": cam_data["camera_matrix"].tolist(),
            "dist_coeff": cam_data["dist_coeffs"].flatten().tolist()
        }
        with open(save_path, "w") as f:
            yaml.safe_dump(data, f)
        print(f"[Calib] Saved YAML to {save_path}")

    @staticmethod
    def load_npz(path):
        data = np.load(path, allow_pickle=True)
        return data