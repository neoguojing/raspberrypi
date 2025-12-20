import numpy as np
import cv2
import os
import glob
import json
import pathlib
import sys
from tqdm import tqdm
import config


input_dir = config.image_path
output_dir = config.tmp_path

nx, ny = getattr(config, "chessboard_cols", 9), getattr(config, "chessboard_rows", 9)
print(f"chessboard_cols={nx}:,chessboard_rows={ny}")

json_path = config.camera_info_path
with open(json_path, 'r') as f:
    Cam_Data = json.load(f)

data = []
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)
for Cam in Cam_Data:
    CamID = Cam['name']
    print(f"Calibrating Camera_{CamID}:")
    if Cam.get('camera_matrix'):
        print(f"Calibrating Camera_{CamID}: 已经计算过，跳过")
        data.append(Cam)
        continue
    # if Cam["resolution"] == [3840, 2160]:
    #     nx,ny = 9,9
    #     print(f"Calibrating Camera_{CamID}: 使用9x9")
    # else:
    #     nx,ny = 8,8
    #     print(f"Calibrating Camera_{CamID}: 使用8x8")
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # 构造棋盘格上每个角点的 3D 世界坐标（假设 Z=0 平面）。
    #    这里棋盘格是 9x9（可根据实际改变）。
    objp = np.zeros((nx*ny,3), np.float32)
    objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    try:
        image_patten = os.path.join(input_dir,f'{CamID}/*.jpg')
        images = glob.glob(image_patten)
        print(f"{len(images)} images found for this camera in {image_patten}.")
    except Exception as e:
          print("path not found", e)
    
    image_output_path = os.path.join(output_dir, CamID)
    # pathlib.Path(path).mkdir(parents=True, exist_ok=True) 
    image_output_path_obj = pathlib.Path(image_output_path)
    image_output_path_obj.mkdir(parents=True, exist_ok=True)

    found = 0
    gray = []
    for i in tqdm(range(len(images))):
        fname = images[i]
        # Here, 10 can be changed to whatever number you like to choose
        img = cv2.imread(fname) # Capture frame-by-frame
        #print(images[im_i])
        # 将每张图片转为灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # 调用 cv2.findChessboardCorners 检测棋盘格角点。
        ret, corners = cv2.findChessboardCorners(gray, (nx,ny), None)
        # If found, add object points, image points (after refining them)
        # 如果成功找到角点：
        # 将世界坐标（objp）和图像坐标（corners2）存起来。
        # 用绿色小方块绘制检测到的角点并保存结果图片。
        # cornerSubPix 用于提高角点检测精度。
        if ret == True:
            objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
            found += 1
            # Draw and display the corners
            has_images = any(f.is_file() and f.suffix.lower() in ["png"] for f in image_output_path_obj.iterdir())
            if has_images:
                print(f"目录 {image_output_path} 已存在图片文件，跳过。")
            else:
                img = cv2.drawChessboardCorners(img, (nx,ny), corners2, ret)
                #cv2.imshow('img', img)
                #cv2.waitKey(500)
                # if you want to save images with detected corners 
                # uncomment following 2 lines and lines 5, 18 and 19
                
                image_name = os.path.join(image_output_path,str(found) + '.png')
                print(f"saved image file:{image_name}")
                cv2.imwrite(image_name, img)

    print("Number of images used for calibration: ", found)
    # When everything done, release the capture
    # cap.release()
    # cv2.destroyAllWindows()
    # calibration

    # ⚠️ 如果没有找到角点，跳过这个相机
    if found == 0 or gray is None:
        print(f"⚠️ No valid calibration data for Camera {CamID}, skipping.")
        data.append(Cam)
        continue

    print("Processing calibration:")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # transform the matrix and distortion coefficients to writable lists
    Cam['camera_matrix'] =  np.asarray(mtx).tolist()
    Cam['dist_coeff'] = np.asarray(dist).tolist()
    data.append(Cam)

# and save it to a file
if data:
    with open(json_path, "w") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
# done