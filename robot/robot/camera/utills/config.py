import os
base_path = ""
input_video_path = os.path.join(base_path,"videos")
image_path = os.path.join(base_path,"Calib_input")
tmp_path = os.path.join(base_path,"Calib_output")
map_image_path = os.path.join(base_path,"map.jpeg")
MAP_WIDTH_FEET = 333500*0.00328                          # 地图宽度（英尺）
MAP_HEIGHT_FEET = 80400*0.00328                         # 地图高度（英尺）
camera_info_path = "camera_info.json"
format_camera_info_path = "camera_info_format.json"
camera_info_split_path = "./pub"

# ⚠️ 注意：这里是内角点数量，不是格子数！
# 例如：棋盘有10x10个格子 => nx=9, ny=9
chessboard_cols = 8
chessboard_rows = 8

camera_ids = []
camera_names = []

camera_types = []

camera_resolution = []

camera_real_xy = {
}