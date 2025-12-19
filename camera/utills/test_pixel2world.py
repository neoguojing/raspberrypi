import calibrator
from Box2xy import undistUV_2_xy
import config
import json

pixel_points = [
    ["L5-3M-4KC-008",3162,362],
]


if __name__ == "__main__":
    json_path = config.camera_info_path
    with open(json_path, 'r') as f:
        Cam_Data = json.load(f)

    for Cam in Cam_Data:
        for p in pixel_points:
            if p[0] == Cam['name']:
                #得到原图像素(u, v)在去畸变图中的像素位置
                u, v = calibrator.uv_2_undist(p[1], p[2], Cam)
                x, y = undistUV_2_xy(u, v, Cam)
                print(x,y)