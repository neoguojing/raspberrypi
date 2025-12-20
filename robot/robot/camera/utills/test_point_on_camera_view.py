
import json
import cv2
import util
import Box2xy
import os
import config

def draw_point_on_image(image_path, u, v):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"无法加载图片: {image_path}")

    pt = (int(round(u)), int(round(v)))

    # 画红色实心圆点，半径5
    cv2.circle(img, pt, radius=5, color=(0, 0, 255), thickness=-1)

    cv2.imwrite(image_path, img)
    print(f"保存结果图片到 {image_path}")

points = [
    # {"name": "silver", "x_ft": 29.01, "y_ft": 6.54},
    {"name": "pink", "x_ft": 16.04, "y_ft": 10.41},
    # {"name": "smallblack", "x_ft": 29.37, "y_ft": 8.59},
    # {"name": "purple", "x_ft": 33.85, "y_ft": 8.99},
    # {"name": "blue", "x_ft": 38.81, "y_ft": 6.25},
]

with open(config.camera_info_path, 'r') as f:
    Cam_Data = json.load(f)

input_image_path = config.tmp_path

for Cam in Cam_Data:
    if Cam['id'] in config.camera_ids:
        for point in points:
            u,v = Box2xy.xy_2_undistOri(point['x_ft'],point["y_ft"],Cam=Cam)
            print(f"{Cam['id']},{point['name']},{u},{v}")
            id = Cam['id']
            image_path = os.path.join(input_image_path,id,f"{id}_view_marked.png")
            if image_path:
                draw_point_on_image(image_path,u,v)

for point in points:
    cameras = util.find_cameras_byPoint(point['x_ft'],point["y_ft"])
    print(f"{point['name']}:{cameras}")
