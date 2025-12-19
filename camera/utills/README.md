<!-- ### ！！！场景视频1-3视频文件放到input/video下
- eg. input/video/1-1-1.mp4, input/video/2-1-1.mp4 .... -->
  
## 生成相机信息json文件
- `bash build_camera_json.sh`
- **Camera_Info.json** 为相机信息生成文件
- ##### **json 文件中的值**
    - **id**[√]: CamID, 相机的id
    - **name**[√]: 相机名称
    - **type**[√]: 相机类型
    - **resolution**[√]: 相机分辨率
    - **ROI_ori**[√]: ROI在相机像素坐标系上的roi，由多边形角点像素坐标表示 (ROI[0]为可构成矩形的左上角点,以下ROI同样符合)
    - **ROI_undist**[√]: ROI在相机去畸变图像坐标系上的roi，由多边形角点像素坐标表示
    - **ROI_grid**[√]: ROI在地图上以方砖角点度量的位坐标，如地图上一点在横坐标第a个方砖角点，纵坐标第b个方砖角点，则存入(a, b)
    - **ROI_xy_by_grid**[√]: 经由ROI_grid通过地图对应关系求得的ROI在实际空间位置(俯视图)中的坐标，由多边形个角点空间坐标表示 单位英尺
    - **ROI_xy**[√]: 映射变换求得的ROI在实际空间位置(俯视图)中的坐标，由多边形个角点空间坐标表示 单位英尺
    - **ROI_rect_ind**[√]: 构成矩形的四点在ROI_grid中的index
    <!-- - **Cam_Pos**: 相机实际空间位置(x, y, z)单位m -->
    - **camera_matrix**[√]: 相机内参矩阵
    - **dist_coeff**[√]: 畸变参数
    <!-- - **rvec**: 相机外参， 旋转向量，通过cv2.Rodrigues转换成旋转矩阵 -->
    <!-- - **tvec**: 相机外参， 平移向量 -->
    - **PerTrans**[√]: 透视变换矩阵， 将去畸变图视角像素一点变换到俯视图视角像素一点 
    <!-- - **mask_path**: 预处理文件，用于 util.find_cameras_byPoint(x, y) 判断 -->

**------------------------------- 运行该指令后可运行下面的指令 -----------------------------------**


<!-- ## 1. 生成轨迹视频：
- `bash run.sh`
- 在**output/trackVideo/**中可找到轨迹视频 -->

<!-- ## 2. 内参和外参精度测量
- `bash eval.sh`
- 信息在终端输出 -->

## 功能函数
1) 像素坐标到地图坐标映射:
- `import Box2xy`
- `Cam_Data = json.load(file_path)`
- `Cam = [cam for cam in Cam_Data if cam['id'] == CamID][0] #给定CamID在json中找到对应的Cam信息字典`
- `Box2xy.transform(u, v, Cam) #给出Cam（Cam为Camera_Info.json文件中对应的字典）原图像素点(u, v), 返回 retval, x, y 对应映射的空间坐标系点(x, y, 0)， 以及该点是否在该Cam的ROI内：-1 不在, 0 在` 
2) util中提供了一些功能函数：
- **point_in_quadrilateral(point, corners)**: 给定一点(x, y)作为point,给出同一坐标系下的多边形的点坐标list作为corners，判断point是否在corners构成的四边形内部，输出True or False
    - ##### Input Paras:
    - **point**: 2D坐标(x, y)
    - **corners**: list of corner points, 需与point坐标系一致
    - ##### Retval:
    - **True or False**: 判断point是否在corners构成的多边形内部
- **find_cameras_byPoint(x, y)**: 输入空间坐标系中一点(x, y, 0)，平地z轴为0， 返回视角内包含该点的相机id list
    - 需要提前生成 Camera_Info.json
    - ##### Input Paras:
    - **x, y** 空间坐标系中点坐标(x, y, 0)， 平地z轴为0 
    - ##### Retval:
    - **list**: 所有视角内包含空间坐标系(x, y, 0)的相机id的list
- **ROI_in_Camera(roi, CamID, scale = 1.0, mode = 0)**: #给定空间坐标系下roi，判断该roi经过scale缩放后是否全部在id为CamID相机的视角内，输出True or False (mod 默认为 0，传入1则roi为相机像素坐标系下的roi)
    - 需要提前生成 Camera_Info.json
    - ##### Input Paras:
    - **roi**: 空间坐标系下多边形角点的list， 当mode(默认为0)设为1时， roi为原相机像素坐标系下像素多边形角点的list
    - **CamID**: 感兴趣的相机的ID
    - **scale**: 默认为 1.0， 缩放系数，会使roi进行scale比例缩放
    - **mode**: 默认为0，控制输入roi的处理方式，0时按照空间坐标系处理， 1时按照像素坐标系处理
    - ##### Retval:
    - **True or False**: 判断经过scale缩放后的roi指定的区域是否全部在id为CamID的相机视角内
- **point_in_ROI(x, y, CamID, scale = 1.0)**: #给出空间中一点(x, y, 0) 判断该点是否在CamID相机的ROI经过scale缩放后的区域内
    - 需要提前生成 Camera_Info.json
    - ##### Input Paras:
    - **x, y**: 空间坐标系下坐标
    - **CamID**: 感兴趣的相机的ID
    - **scale**: 默认为 1.0， 缩放系数，会使roi进行scale比例缩放
    - ##### Retval:
    - **True or False**

