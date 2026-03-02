# yolo 模块说明

`yolo/` 存放视觉分割与感知实验代码，重点是将图像分割结果转换为导航可用信息。

## 文件结构

- `zen_seg.py`：订阅图像流，执行分割并生成伪 `LaserScan` 思路数据。
- `test_on_ros2.py`：ROS2 场景下的测试脚本。
- `test_on_yolo.py`：YOLO 推理测试脚本。
- `test_pixel_inverse_consistency.py`：像素几何一致性验证脚本。

## 说明

该目录偏实验性质，运行前请确认模型依赖、设备加速环境与输入数据源已准备。
