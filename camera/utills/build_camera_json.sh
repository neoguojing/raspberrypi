# 建议一步一步执行

# 1.Initialize camera_Info.json : 初始化待处理相机的id, name, type
# 按需更改相机的分辨率大小，JSON_PATH也可更改
python jsonInitializer.py

# 2.Get frame images from videos so calibration can be performed
# 修改sh里的base_path(视频文件夹)
# bash FrameImgs_Extractor.sh
python extract_frames.py

# 3.Camera Calibration and update json file :  将相机内参和畸变参数写入json文件 
# 更改py里的input_dir(保存的帧图)和output_dir（绘制的帧图）
# 依据视频中的棋盘的尺度，计算像素点和现实世界的尺度关系；输出：内参矩阵，畸变参数等
python calibration_script.py 1
# 摄像头和地图标注
python mark_on_camera_view_pix.py
# 4.update ROI info : 将透视变换所用的ROI和矩阵写入json
# 更改py里面dict_roi，按照提供的说明来改
# 输入：摄像头的roi 和 棋盘roi，输出：透视矩阵
python pertrans.py

# 5.calculate mapping : 将地图坐标下ROI_xy写入json
python Box2xy.py

# 6.修缮json文件格式: 最终文档 JSON_PATH.json
python postProcessing.py