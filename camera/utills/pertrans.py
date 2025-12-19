import cv2
import numpy as np
import calibrator
import json
import sys
import pathlib
import config
import jsonInitializer

def eval(CamID):
    """
    对比某个相机的 undistorted ROI 与预设观测点 roi_ob 的坐标差异。

    参数:
        CamID (str or int): 相机的唯一标识符，用于从 dict_roi 中取出该相机的 ROI 信息。

    功能:
        - 打印每对 ROI 点之间的欧氏距离（L2范数）。
        - 输出格式为：roi_ob坐标, roi_undist坐标, 二者之间的距离。

    依赖:
        - dict_roi: 字典，包含每个相机的 ROI_undist 数据。
        - OpenCV 和 NumPy。
    """
    
    roi_undist = dict_roi[CamID]['ROI_undist']
    roi_ob = [[648, 884], [1074, 483], [2084, 543], [2242, 1029]]
    for i in range(len(roi_undist)):
        print(roi_ob[i], list(roi_undist[i]), cv2.norm(np.array(roi_ob[i], dtype = np.float64), np.array(roi_undist[i], dtype = np.float64), cv2.NORM_L2))

def get_Rect(ROI_grid): # 假设ROI_grid[0]定制为能构成内切四边形的左上角点, 给定多边形ROI，放回返回能构成矩形的四点的index
    """
    从多边形顶点中提取能构成内切矩形的四个点的索引。

    参数:
        ROI_grid (List[List[int]]): 多边形的网格点坐标列表，格式为 [(x1, y1), (x2, y2), ...]。
            假设第一个点为左上角，寻找能构成矩形的其他三个点。

    返回:
        List[int]: 构成矩形四个点的索引，按顺时针顺序排列：[左上, 右上, 右下, 左下]。
        如果无法找到完整矩形，则只返回包含左上角的部分列表。
    
    注意:
        - 要求输入点中必须存在一个规则矩形的四个顶点。
    """
    a0, b0 = ROI_grid[0]
    lst_a = []
    lst_b = []
    result = [0]
    n = len(ROI_grid)
    for i in range(1, n):
        a, b = ROI_grid[i]
        if a == a0:
            lst_a.append(i)
        if b == b0:
            lst_b.append(i)
    for i_a in lst_a:
        a1, b1 = ROI_grid[i_a]
        for i_b in lst_b:
            a2, b2 = ROI_grid[i_b]
            pt = (a2, b1)
            find = [ind for ind, ptd in enumerate(ROI_grid) if (ptd[0] == pt[0] and ptd[1] == pt[1])]
            if len(find) >= 1:
                # 按照左上，右上，右下，左下的顺序存储
                result.append(i_b)
                result.append(find[0])
                result.append(i_a)
                return result
    return result

def get_rect_sorted_points(pts):
    """
    输入任意顺序的四边形点，返回顺时针排列的矩形4点。

    参数:
        pts (List[Tuple[int, int]]): 输入的四边形顶点，至少4个点

    返回:
        rect_pts (List[Tuple[int, int]]): 顺时针排列的矩形4个点
                                          格式: [左上, 右上, 右下, 左下]
    """
    if len(pts) < 4:
        raise ValueError("至少需要4个点")

    # 将输入转换为 NumPy array
    pts_array = np.array(pts, dtype=np.float32)

    # 用 OpenCV 拟合最小外接矩形（旋转包围框）
    rect = cv2.minAreaRect(pts_array)  # 返回中心点、尺寸、旋转角度
    box = cv2.boxPoints(rect)          # 返回4个角点
    box = np.array(sorted(box.tolist()), dtype=np.float32)

    # 按照从左上角开始顺时针排序
    return order_points_clockwise(box)


def order_points_clockwise(pts):
    """
    给定4个点，返回顺时针排序（左上、右上、右下、左下）

    参数:
        pts (np.ndarray): (4, 2) 的数组

    返回:
        rect (List[Tuple[int, int]]): 顺时针排序的点
    """
    rect = np.zeros((4, 2), dtype="float32")

    # 和 = 左上，差 = 右上
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    rect[0] = pts[np.argmin(s)]      # 左上
    rect[2] = pts[np.argmax(s)]      # 右下
    rect[1] = pts[np.argmin(diff)]   # 右上
    rect[3] = pts[np.argmax(diff)]   # 左下

    return rect.tolist()

def get_H_pixel_world(Cam):
    """
    计算像素坐标 <-> 世界真实坐标（英尺）的单应矩阵

    需要 Cam 包含:
        ROI_undist: 去畸变后的 ROI 像素点（4点）
        ROI_real: 对应的世界坐标（英尺）
    """
    roi_rect_ind = [0,1,2,3]

    pts_pixel = np.float32(Cam['ROI_undist'])   # shape: (4,2)
    pts_world = np.float32(Cam['ROI_real'])     # shape: (4,2)

    # 像素 → 世界坐标（英尺）
    H_pixel2world = cv2.getPerspectiveTransform(pts_pixel, pts_world)

    # 世界 → 像素
    H_world2pixel = np.linalg.inv(H_pixel2world)

    return H_pixel2world, roi_rect_ind

def get_PerTrans(Cam):
    """
    计算相机 ROI 的透视变换矩阵（Perspective Transform Matrix）。

    参数:
        Cam (dict): 包含相机信息的字典，需包含以下键：
            - 'ROI_undist': 去畸变后的 ROI 点坐标，长度为4。
            - 'ROI_grid': 原始 ROI 网格坐标（整数点）。

    返回:
        Tuple[np.ndarray, List[int]]:
            - 透视变换矩阵 (3x3 numpy array)
            - 构成矩形的四个 ROI_grid 点索引（按顺时针排列）。

    依赖:
        - get_Rect
        - jsonInitializer.grid_2_xy
        - get_ROI_wh
        - get_pertrans_WH
        - OpenCV

    功能:
        - 根据 ROI 点计算矩形宽高。
        - 构造目标平面上的点。
        - 调用 OpenCV 生成透视变换矩阵。
    """
    # 原图中书本的四个角点(左上、右上、右下、左下),与变换后矩阵位置,排好序的角点输出，0号是左上角，顺时针输出
    roi = Cam['ROI_undist']
    # 不使用grid标注的情况下，直接返回了实际坐标
    roi_rect_ind = [0,1,2,3]
    roi_rect = []
    if Cam.get('ROI_grid'):
        roi_rect_ind = get_Rect(Cam['ROI_grid'])
        assert len(roi_rect_ind) == 4
        roi_rect = [jsonInitializer.grid_2_xy(*Cam['ROI_grid'][i]) for i in roi_rect_ind]
    else:
        roi_rect = get_rect_sorted_points(Cam.get('ROI_real'))

    wx, hx = get_ROI_wh(roi_rect)
    w, h = get_pertrans_WH((wx, hx))
    pts1 = np.float32(roi)
    #变换后矩阵位置
    pts2 = np.float32([[0, 0],[w, 0],[w, h],[0, h],])
    # 生成透视变换矩阵；进行透视变换
    m = cv2.getPerspectiveTransform(pts1, pts2)
    return m, roi_rect_ind

def get_ROI_wh(ROI_xy):
    """
    计算 ROI 矩形区域的宽度和高度。

    参数:
        ROI_xy (List[List[float]]): 四个 ROI 点坐标（x, y），表示一个矩形。

    返回:
        Tuple[float, float]: ROI 的宽度和高度 (width, height)，单位与输入坐标一致。
    """
    pt_min = [9999999.0, 9999999.0]
    pt_max = [0.0, 0.0]
    for x, y in ROI_xy:
        pt_min[0] = min(pt_min[0], x)
        pt_min[1] = min(pt_min[1], y)
        pt_max[0] = max(pt_max[0], x)
        pt_max[1] = max(pt_max[1], y)
    return (pt_max[0] - pt_min[0], pt_max[1] - pt_min[1])

def get_pertrans_WH(ROI_wh):
    """
    将 ROI 实际宽高转换为透视变换后目标图像尺寸。

    参数:
        ROI_wh (Tuple[float, float]): 原始 ROI 宽度和高度。

    返回:
        Tuple[int, int]: 变换后图像的宽度和高度，单位为像素。
        计算方式为：将原始宽高乘以 50 并四舍五入后乘以 2。
    """
    w = 2 * round(ROI_wh[0] * 50)
    h = 2 * round(ROI_wh[1] * 50)
    return (w, h)


if __name__ == "__main__":
    camera_ids = config.camera_ids
    json_path = config.camera_info_path
    with open(json_path, "r") as f:
        Cam_Data = json.load(f)
    try:
        args = sys.argv[1]
    except Exception:
        args = '0'
    if args == '1':
        input_dir = 'input/Calib'
        output_dir = "output/overlook"
        Cam = Cam_Data[camera_ids[0]]
        CamID = Cam['id']
        img = cv2.imread(input_dir + f'/Cam_{CamID}.jpg')
        W_cols, H_rows= img.shape[:2]
        print("img_Size :", W_cols, 'x', H_rows)

        m, _ = get_PerTrans(Cam)
        dst = cv2.warpPerspective(img, m, get_pertrans_WH(Cam['ROI_wh']))
        
        pathlib.Path(output_dir).mkdir(parents=True, exist_ok=True)
        output_path = output_dir + f"/overlook-1-{id}.jpg"
        cv2.imwrite(output_path, dst)
        eval(CamID)
    else:
        data = []
        for Cam in Cam_Data:
            #del Cam['ROI_wh']
            id = Cam['id']
            roi_undist = []
            if Cam.get('ROI_ori'):
                for u, v in Cam['ROI_ori']:
                    roi_undist.append(calibrator.uv_2_undist(u, v, Cam))
                # 去畸变坐标
                Cam['ROI_undist'] = [[u, v] for u, v in roi_undist]
            # 真实世界坐标
            Cam['ROI_xy_by_grid'] = []
            if Cam.get('ROI_grid'):
                for a, b in Cam.get('ROI_grid'):
                    Cam['ROI_xy_by_grid'].append(jsonInitializer.grid_2_xy(a, b))
            
            if Cam.get('ROI_real'):
                Cam['ROI_xy_by_grid'] = get_rect_sorted_points(Cam.get('ROI_real'))

            # 透视变换矩阵
            if Cam.get('ROI_undist') and (Cam.get('ROI_real') or Cam.get('ROI_grid')):
                # m, roi_rect_ind = get_PerTrans(Cam)
                m, roi_rect_ind = get_H_pixel_world(Cam)
                Cam['ROI_rect_ind'] = roi_rect_ind
                Cam['PerTrans'] = np.array(m).tolist()
            data.append(Cam)
        if data:
            with open(json_path, 'w') as f:
                json.dump(data, f, ensure_ascii=False, indent=4)
    