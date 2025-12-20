import os
import cv2
import json
import numpy as np
import config
# === 参数设置 ===
map_path = config.map_image_path
output_dir = "output_map"                       # 输出绘制目录
json_path = "roi_annotations_on_map.json"       # JSON 输出路径
MAP_WIDTH_FEET = config.MAP_WIDTH_FEET                           # 地图宽度（英尺）
MAP_HEIGHT_FEET = config.MAP_HEIGHT_FEET                         # 地图高度（英尺）


def annotate_on_map(cam_id):
    # 加载图像
    original = cv2.imread(map_path)
    assert original is not None, f"图像加载失败: {map_path}"
    img_h, img_w = original.shape[:2]

    # 英尺/像素比例
    feet_per_pixel_x = MAP_WIDTH_FEET / img_w
    feet_per_pixel_y = MAP_HEIGHT_FEET / img_h

    print(f"地图尺寸: {img_w}px x {img_h}px 对应 {MAP_WIDTH_FEET}ft x {MAP_HEIGHT_FEET}ft")
    print(f"每像素对应尺寸: x={feet_per_pixel_x:.4f} ft, y={feet_per_pixel_y:.4f} ft")

    # 标注状态
    display_img = original.copy()
    current_points = []
    roi_groups = []

    def draw_point_with_index(img, point, index):
        """在图像上绘制点和序号"""
        cv2.circle(img, point, 5, (0, 0, 255), -1)  # 红色圆点
        cv2.putText(img, str(index), (point[0] + 5, point[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)  # 黄色编号

    def mouse_callback(event, x, y, flags, param):
        nonlocal current_points, display_img

        if event == cv2.EVENT_LBUTTONDOWN:
            current_points.append((x, y))

            # 画点和编号
            draw_point_with_index(display_img, (x, y), len(current_points))

            # 连线
            if len(current_points) > 1:
                cv2.line(display_img, current_points[-2], current_points[-1], (0, 0, 255), 2)

    # 注册鼠标事件
    cv2.namedWindow("Map Annotator", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Map Annotator", mouse_callback)

    print(f"\n🖼️ 开始标注地图（点击任意点，Enter 结束一个 ROI）")
    print("👉 左键点击绘制点，按 Enter 确认 ROI，按 Q 保存退出，ESC 清除所有")

    while True:
        cv2.imshow("Map Annotator", display_img)
        key = cv2.waitKey(1)

        if key in [13, 10]:  # Enter：结束当前 ROI
            if len(current_points) >= 3:
                # 闭合最后一条边
                cv2.line(display_img, current_points[-1], current_points[0], (0, 0, 255), 2)
                roi_groups.append(current_points.copy())
                print(f"✅ 完成 ROI #{len(roi_groups)}: {roi_groups}")
                current_points.clear()
                break
            else:
                print("⚠️ 至少需要 3 个点构成一个 ROI，当前点数不足")

        elif key == 27:  # ESC 清空
            roi_groups.clear()
            current_points.clear()
            display_img = original.copy()
            print("⚠️ 已清除所有 ROI")

    cv2.destroyAllWindows()

    # 输出 JSON（仅 feet 坐标）
    feet_points = []
    for i, group in enumerate(roi_groups):
        feet_points = [
            (
                round(x * feet_per_pixel_x, 2),
                round(y * feet_per_pixel_y, 2)
            ) for x, y in group
        ]

    return f"{cam_id}_map_marked.png",feet_points,display_img
