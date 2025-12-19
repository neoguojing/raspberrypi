import numpy as np
import cv2
import glob
import json
import pathlib
import sys
import jsonInitializer
from tqdm import tqdm
    
def get_UndistParas(Cam):
    mtx = np.asarray(Cam['camera_matrix'])
    dist = np.asarray(Cam['dist_coeff'])
    w = h = 0
    if isinstance(Cam['resolution'],dict):
        w, h = Cam['resolution'].get("width"),Cam['resolution'].get("height")
    else:
        w, h = Cam['resolution']

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    return mtx, dist, newcameramtx, roi

def uv_2_undist(u, v, Cam):
    mtx, dist, n_mtx, roi = get_UndistParas(Cam)
    x0, y0, _, _ = roi
    dst = cv2.undistortPoints((u, v), mtx, dist, None, n_mtx)
    u, v = dst[0][0]
    u -= x0
    v -= y0
    return u, v

def uv_2_undistOri(u, v, Cam):
    mtx, dist, n_mtx, _ = get_UndistParas(Cam)
    dst = cv2.undistortPoints((u, v), mtx, dist, None, n_mtx)
    u, v = dst[0][0]
    return u, v

if __name__ == "__main__":
    # 系统输入：CamID, 输入路径, 输出路径
    json_path = jsonInitializer.JSON_PATH
    with open(json_path, 'r') as f:
        Cam_Data = json.load(f)
    args = sys.argv[1:]
    CamID = int(args[0])
    input_dir = args[1]
    output_dir = args[2]
    Cam = [cam for cam in Cam_Data if cam['id'] == CamID][0]
    mtx, dist, newcameramtx, roi = get_UndistParas(Cam)
    path = input_dir + "/*.jpg"
    try:
        images = glob.glob(path)
        images.sort()
    except Exception as e:
        print(f"path failed : {e}")
    outputPath_unclipped = output_dir + "/unclipped"
    outputPath_clipped = output_dir + "/clipped"
    pathlib.Path(outputPath_unclipped).mkdir(parents=True, exist_ok=True)
    pathlib.Path(outputPath_clipped).mkdir(parents=True, exist_ok=True)
    for i in tqdm(range(len(images))):
        img = cv2.imread(images[i])
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        outputPath = outputPath_unclipped + f"/{i+1:05d}.jpg"
        cv2.imwrite(outputPath, dst)
        dst = dst[y:y+h, x:x+w]
        outputPath = outputPath_clipped + f"/{i+1:05d}.jpg"
        cv2.imwrite(outputPath, dst)