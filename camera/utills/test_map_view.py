import cv2
import numpy as np
import json
import jsonInitializer
import config
import os


# 4. 实际尺寸（单位：英尺）
feet_width = config.MAP_WIDTH_FEET
feet_height = config.MAP_HEIGHT_FEET

def get_image(image_path):
    # 3. 地图图像
    image = cv2.imread(image_path)
    height, width, _ = image.shape

    print(width,height)
    
    scale_x = width / feet_width
    scale_y = height / feet_height
    return image,scale_x,scale_y

# 5. 坐标转换
def feet_to_pixel(feet_x, feet_y, scale_x, scale_y):
    x_px = int(feet_x * scale_x)
    y_px = int(feet_y * scale_y)  # 去掉 img_height 的反转
    return x_px, y_px


def blend_roi(image, polygon_pts, color, alpha=0.4):
    """
    将一个 ROI 多边形以指定透明度叠加到原图上
    """
    overlay = image.copy()
    cv2.fillPoly(overlay, [polygon_pts], color)
    return cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

def draw_roi_grid(roi_grid,image,scale_x, scale_y):
    # 绘制转换后的坐标
    pixel_points = [feet_to_pixel(*jsonInitializer.grid_2_xy(p["x"], p["y"]), scale_x, scale_y) for p in roi_grid]
    contour = np.array(pixel_points, dtype=np.int32).reshape((-1, 1, 2))

    # 叠加当前 ROI 区域（带透明度）
    image = blend_roi(image, contour, (0, 255, 0), alpha=0.4)
    return image

def draw_roi_map(roi_map,image,scale_x, scale_y):
        # 绘制转换后的坐标
    pixel_points = [feet_to_pixel(p["x"], p["y"], scale_x, scale_y) for p in roi_map]
    contour = np.array(pixel_points, dtype=np.int32).reshape((-1, 1, 2))

    # 叠加当前 ROI 区域（带透明度）
    image = blend_roi(image, contour, (0, 255, 0), alpha=0.4)
    return image

output_base = config.tmp_path
Cam_Data = None
with open(config.format_camera_info_path, "r") as f:
    Cam_Data = json.load(f)
for Cam in Cam_Data:
    if len(Cam) < 13:
        continue
    roi_mark = Cam.get("ROI_real")
    roi_grid = Cam.get("ROI_grid")
    roi_format = Cam.get("ROI_xy_by_grid")
    roi_final = Cam.get("ROI_xy")
    id = Cam.get("name")
    if roi_final:
        # 6. 绘制所有 ROI
        image_path = os.path.join(output_base,id,f"{id}_map_marked.png")
        image,scalex,scaley = get_image(config.map_image_path)
        # image = draw_roi_map(name,roi_mark,image)
        # image = draw_roi_map(name,roi_format,image)
        final_image = draw_roi_map(roi_final,image,scalex,scaley)
        cv2.imwrite(image_path, final_image)
    # if roi_grid:
    #     image = draw_roi_grid(name,roi_grid,image)
    #     image = draw_roi_map(name,roi_final,image)


