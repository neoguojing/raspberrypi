import json
import cv2
import numpy as np
import calibrator
import jsonInitializer
import pertrans
import util
import config
# keys = ["frame_index", "x1", "y1", "x2", "y2", "label"]
# workdir = "input/BoundingBox/clean/"

def undistUV_2_xy(u, v, Cam): # 给出去在CamID相机视角下畸变图像上像素一点(u, v)，返回对应的空间坐标系坐标(x, y , 0)
    m = np.array(Cam['PerTrans'])
    # 计算出在去畸变图上(u, v)坐标的像素在俯视图上映射的像素坐标(x, y)
    dst = np.dot(m, np.array([u, v, 1]).T)
    dst /= dst[2]
    x, y = dst[:2]
    # 根据多边形内接矩形进行映射（单位英尺）
    roi_rect_ind = Cam['ROI_rect_ind']
    wROI, hROI = pertrans.get_ROI_wh(ROI_xy=[Cam['ROI_xy_by_grid'][ind] for ind in roi_rect_ind])
    pw, ph = pertrans.get_pertrans_WH((wROI, hROI))
    # ROI左上角在地图的位置
    x0, y0 = Cam['ROI_xy_by_grid'][roi_rect_ind[0]]
    # 左下为0点
    # return ( x0 + 1.0 * x * (wROI / pw), y0 - 1.0 * y * (hROI / ph) )
    # 左上为0点
    return ( x0 + 1.0 * x * (wROI / pw), y0 + 1.0 * y * (hROI / ph) )
 

def xy_2_undistUV(x, y, Cam): # 给出空间坐标系坐标(x, y, 0) 返回对应在裁剪后的去畸变图像上的像素坐标
    # 根据多边形内接矩形进行映射（单位英尺）
    roi_rect_ind = Cam['ROI_rect_ind']
    wROI, hROI = pertrans.get_ROI_wh(ROI_xy=[Cam['ROI_xy_by_grid'][ind] for ind in roi_rect_ind])
    pw, ph = pertrans.get_pertrans_WH((wROI, hROI))
    x0, y0 = Cam['ROI_xy_by_grid'][roi_rect_ind[0]]
    # 原地为图像左下角
    # u, v = ( 1.0 * (x - x0) * (pw / wROI), 1.0 * (y0 - y) * (ph / hROI))
    # 原地为图像左上角
    u, v = ( 1.0 * (x - x0) * (pw / wROI), 1.0 * (y - y0) * (ph / hROI))
    # 俯视图像素坐标到去畸变图坐标
    m = np.array(Cam['PerTrans'])
    m_inv = np.linalg.inv(m)
    dst = np.dot(m_inv, np.array([u, v, 1]).T)
    dst /= dst[2]
    return dst[:2]

def xy_2_undistOri(x, y, Cam): # 给出空间坐标系坐标(x, y, 0) 返回对应在未裁剪的去畸变图像上的像素坐标
    # 找到去畸变图坐标
    u, v = xy_2_undistUV(x, y, Cam)
    _, _, _, roi = calibrator.get_UndistParas(Cam)
    x0, y0, _, _ = roi
    u += x0
    v += y0
    return u, v
    

def transform(u, v, Cam): #输入在畸变图像上的像素坐标，输出()在平面俯视图上的坐标（英尺）
    #得到原图像素(u, v)在去畸变图中的像素位置
    u, v = calibrator.uv_2_undist(u, v, Cam)
    
    roi_undist = Cam['ROI_undist']
    retval = 0
    if util.point_in_quadrilateral((u, v), roi_undist) == False:
        retval = -1
    x, y = undistUV_2_xy(u, v, Cam)
    #if util.point_in_quadrilateral((x, y), Roi_xy) == False:
    #    retval = -1
    return retval, x, y

# def work(input_path, output_path, CamID):
#     data = []
#     store_data = []
#     with open(input_path, "r") as f:
#         data = json.load(f)
#     data = [dict for dict in data if dict[keys[5]] != 37017] # 筛掉人脸box"37017""
#     for dict in data:
#         # For each of the box dict, work out the esti-(u, v) in the distorted img:
#         x1 = dict["x1"]
#         x2 = dict["x2"]
#         y2 = dict["y2"]
#         u, v = ((x1 + x2) / 2, y2)
#         # 得到俯视图坐标
#         retval, x, y = transform(u, v, CamID)
#         if retval == -1: #在ROI外不考虑
#             continue
#         position = (x, y)
#         # 存储新数据
#         tar = {}
#         tar['position'] = position
#         tar['frame_index'] = dict[keys[0]]
#         if dict[keys[5]] == 221488:
#             tar['label'] = "H" 
#         if dict[keys[5]] == 200814:
#             tar['label'] = "L"
#         store_data.append(tar)

#     with open(output_path, "w") as f:
#         json.dump(store_data, f, ensure_ascii=False, indent=4)

if __name__ == "__main__":
    # for SceneID in range(1, 4):
    #     for CamID in range(1, 3):
    #         for VideoID in range(1, 4):
    #             input_path = workdir + f"clean_{SceneID}_{CamID}_{VideoID}.json"
    #             output_path = workdir + f"Box2xy_{SceneID}_{CamID}_{VideoID}.json"
    #             work(input_path, output_path, CamID)
    #             print(f"Done for {SceneID}_{CamID}_{VideoID}")
    
    json_path = config.camera_info_path
    with open(json_path, 'r') as f:
        Cam_Data = json.load(f)

    data = []
    for Cam in Cam_Data:
        ROI_xy = []
        if Cam.get('ROI_ori'):
            for u, v in Cam['ROI_ori']:
                x, y = transform(u, v, Cam)[1:]
                ROI_xy.append([round(x, 2), round(y, 2)])
            Cam['ROI_xy'] = ROI_xy
        data.append(Cam)
    if data:
        with open(json_path, 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
