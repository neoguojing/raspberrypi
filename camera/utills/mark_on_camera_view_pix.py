import os
import cv2
import json
import numpy as np
import config
import mark_on_map_inch

def annotate_image(image_path, image_name):
    points = []

    original = cv2.imread(image_path)
    h, w = original.shape[:2]

    # 设置最大窗口尺寸
    max_win_w, max_win_h = 1280, 720
    scale = min(max_win_w / w, max_win_h / h, 1.0)

    # 缩放图像用于显示
    resized = cv2.resize(original.copy(), (int(w * scale), int(h * scale)))
    display_img = resized.copy()

    def mouse_callback(event, x, y, flags, param):
        nonlocal display_img
        if event == cv2.EVENT_LBUTTONDOWN:
            real_x = int(x / scale)
            real_y = int(y / scale)
            points.append((real_x, real_y))

            # 在缩放图上画点和连线
            cv2.circle(display_img, (x, y), 5, (0, 0, 255), -1)
            if len(points) > 1:
                x1, y1 = int(points[-2][0] * scale), int(points[-2][1] * scale)
                x2, y2 = int(points[-1][0] * scale), int(points[-1][1] * scale)
                cv2.line(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 序号标注（缩放图）
            point_num = len(points)
            cv2.putText(display_img, str(point_num), (x + 5, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.namedWindow("Annotate", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Annotate", mouse_callback)

    print(f"\n🖼️ 标注图像: {image_name}")
    print("   👉 左键点击添加点，Enter 完成，ESC 清除，S 跳过")

    while True:
        cv2.imshow("Annotate", display_img)
        key = cv2.waitKey(10)
        if key in [13, 10]:  # Enter 确认
            break
        elif key == 27:  # ESC 清除
            points.clear()
            display_img = resized.copy()
            print("   ⚠️ 已清除所有点，请重新标注")
        elif key == ord('s'):  # 跳过
            print("   ⚠️ 跳过当前图像")
            return None, None,None

    cv2.destroyAllWindows()

    # === 绘制最终 ROI 到原图上，并添加编号 ===
    final_img = original.copy()
    if len(points) >= 3:
        pts_np = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
        overlay = final_img.copy()
        cv2.fillPoly(overlay, [pts_np], (0, 255, 0))  # 绿色填充
        final_img = cv2.addWeighted(overlay, 0.4, final_img, 0.6, 0)

    # 添加点编号和红点
    for idx, (x, y) in enumerate(points):
        cv2.circle(final_img, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(final_img, str(idx + 1), (x + 5, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    return image_name, points,final_img

# === 主流程 ===
image_base_dir = config.image_path
out_base_path = config.tmp_path
json_path = config.camera_info_path
with open(json_path, 'r') as f:
    Cam_Data = json.load(f)
data = []
for cam in Cam_Data:
    cam_id = cam['name']
    img_path = os.path.join(image_base_dir, cam_id,"00001.jpg")
    if not os.path.exists(img_path):
        data.append(cam)
        print(f"✅ {img_path} 图片不存在")
        continue
    name, points,final_img = annotate_image(img_path, f"{cam_id}_view_marked.png")
     # 保存绘制后的图像
    if name is not None:
        out_path = os.path.join(out_base_path, cam_id,name)
        cv2.imwrite(out_path, final_img)
        print(f"✅ 已保存摄像头标注图像: {out_path}")
        cam['ROI_ori'] = points

    # name, points,final_img = mark_on_map_inch.annotate_on_map(cam_id)
    # out_path = os.path.join(out_base_path, cam_id,name)
    # cv2.imwrite(out_path, final_img)
    # print(f"✅ 已保存地图标注图像: {out_path}")
    # cam['ROI_real'] = points
    real_xy = config.camera_real_xy.get(cam_id)
    if real_xy:
        cam['ROI_real'] = real_xy
    data.append(cam)

# 保存 JSON 标注文件（像素坐标）
if data:
    with open(json_path, "w") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
    print(f"\n✅ 所有标注完成，JSON 文件已保存到: {json_path}")
