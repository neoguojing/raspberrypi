import json
import cv2
import Box2xy
import calibrator
import jsonInitializer
import config


def point_in_quadrilateral(point, corners): # 给定一点(x, y)作为point,给出同一坐标系下的四边形4点list作为corners， 输出True or False
    def cross_product(x0, y0, x1, y1, x2, y2):
        """ Calculate cross product (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0) """
        return (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
    px, py = point
    pts = []
    for x, y in corners:
        pts.append({
            'x' : x,
            'y' : y
        })
    # Calculate cross products for each triangle formed by the point and the quadrilateral vertices
    cp_results = []
    n = len(pts)
    for i in range(n):
        pt1 = pts[i]
        pt2 = pts[(i + 1) % n]
        cp_results.append(cross_product(px, py, pt1['x'], pt1['y'], pt2['x'], pt2['y']))
    flag_1 = sum(1 for cp in cp_results if cp >= 0)
    flag_2 = sum(1 for cp in cp_results if cp <= 0)
    # Check if the point is inside the quadrilateral (or on its boundary)
    if flag_1 == n or flag_2 == n:
        return True
    else:
        return False


def find_cameras_byPoint(x, y): #输入空间坐标系中一点(x, y, 0)，平地z轴为0， 返回视角内包含该点的相机id list
    json_path = config.camera_info_path
    with open(json_path, "r") as f:
        Cam_Data = json.load(f)
    ret = []
    for Cam in Cam_Data:
        CamID = Cam['id']
        # 把世界坐标映射到去未裁剪畸变图坐标(u, v)
        u, v = Box2xy.xy_2_undistOri(x, y, Cam)
        u = round(u)
        v = round(v)
        _, _, _, roi = calibrator.get_UndistParas(Cam)
        x0, y0, w, h = roi
        roi = [[x0, y0], [x0 + w, y0], [x0 + w, y0 + h], [x0, y0 + h]]
        if point_in_quadrilateral((u, v), roi):
            ret.append(CamID)
    return ret

def scale_boundary(pts, scale):
    if scale == 1.0:
        return pts
    center = [0, 0]
    for pt in pts:
        center[0] += pt[0]
        center[1] += pt[1]
    center = [center[0] / len(pts), center[1] / len(pts)]
    deltas = [[scale * (pt[0] - center[0]), scale * (pt[1] - center[1])] for pt in pts]
    return [(center[0] + u, center[1] + v) for u, v in deltas]

def ROI_in_Camera(roi, CamID, scale = 1.0, mode = 0): #给定roi 判断该roi是否全部在id为CamID相机的视角内
    # roi 为未去畸变图上像素
    json_path = jsonInitializer.JSON_PATH
    with open(json_path, "r") as f:
        Cam_Data = json.load(f)
    try:
        Cam = [cam for cam in Cam_Data if cam['id'] == CamID]
    except Exception:
        print("void key input")
    if mode == 1: # mode = 1 时输入为原始图片像素点， 默认 mode = 0 时为实际空间点坐标
        roi_undist = [Box2xy.transform(u, v, Cam)[1:] for u, v in roi]
        roi_undist = scale_boundary(roi_undist, scale)
    elif mode == 0:
        roi_undist = scale_boundary(roi, scale)
    #print(roi_undist)
    for x, y in roi_undist:
        Cams = find_cameras_byPoint(x, y)
        print(Cams)
        if CamID not in Cams:
            return False
    return True
    
def point_in_ROI(x, y, CamID, scale = 1.0): # 给出空间中一点(x, y, 0) 判断该点是否在CamID相机的ROI经过scale缩放后的区域内
    json_path = jsonInitializer.JSON_PATH
    with open(json_path, "r") as f:
        Cam_Data = json.load(f)
    try:
        Cam = [cam for cam in Cam_Data if cam['id'] == CamID]
    except Exception:
        print("void key input")
    roi = scale_boundary(Cam['ROI_xy'], scale)
    return point_in_quadrilateral((x, y), roi)
if __name__ == "__main__":
    # mode = int(input("choose the fun to be tested:"))
    print("test for find_cameras_byPoint:")
    print("输入以格子为单位的坐标:")
    u = float(input("u = "))
    v = float(input("v = "))
    u = u * 2.887
    v = 1.83 + (v - 1) * 2
    print(find_cameras_byPoint(u, v))