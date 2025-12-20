
import json
import cv2
import numpy as np
import calibrator
import jsonInitializer
import os
import config


def draw_roi_on_image(image_path, roi_points, output_dir=None):
    # 读取图片
    img = cv2.imread(image_path)
    if img is None:
        # raise FileNotFoundError(f"无法加载图片: {image_path}")
        return None

    # 转换 ROI 点为整数 np 数组
    pts = np.array(roi_points, dtype=np.int32).reshape((-1, 1, 2))

    # 在图片上绘制多边形，蓝色线条，宽度3
    cv2.polylines(img, [pts], isClosed=True, color=(255, 0, 0), thickness=3)

    # 显示图片（可选）
    # cv2.imshow('Image with ROI', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # 保存图片（如果指定了保存路径）
    if output_dir:
        cv2.imwrite(output_dir, img)
        print(f"保存结果图片到 {output_dir}")
    return output_dir


with open(config.camera_info_path, 'r') as f:
    Cam_Data = json.load(f)

input_image_path = config.tmp_path
for Cam in Cam_Data:
    if len(Cam) < 13:
        continue
    id = Cam['name']
    image_path = os.path.join(input_image_path,id,f"{id}_view_marked.png")
    if image_path and Cam.get('camera_matrix'):
        _, _, _, roi = calibrator.get_UndistParas(Cam)
        x0, y0, w, h = roi
        roi = [[x0, y0], [x0 + w, y0], [x0 + w, y0 + h], [x0, y0 + h]]
        print(f"{Cam['id']}:{roi}")
        # 摄像头视野去畸变后的视野
        image_path = draw_roi_on_image(image_path,roi,image_path)
        # 标注roi去畸变后的roi
        image_path = draw_roi_on_image(image_path,Cam["ROI_undist"],image_path)

        # image_path = draw_roi_on_image(image_path,Cam["ROI_undist"],"tests")
