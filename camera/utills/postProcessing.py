import json
import jsonInitializer
import config
import os
json_path = config.camera_info_path
with open(json_path, 'r') as f:
    Cam_Data = json.load(f)

update_keys = ['ROI_ori', 'ROI_grid', 'ROI_undist', 'ROI_xy', 'ROI_xy_by_grid','ROI_real']
pair_keys = ['resolution']

for Cam in Cam_Data:
    for key in pair_keys:
        w, h = Cam[key]
        if not isinstance(Cam[key],dict):
            Cam[key] = {
                'width' : w,
                'height' : h
            }
    for key in update_keys:
        data = []
        if Cam.get(key):
            for u, v in Cam[key]:
                data.append({
                    'x' : u,
                    'y' : v
                })
            Cam[key] = data
    if len(Cam) >= 13:
        out_path = os.path.join(config.camera_info_split_path,Cam["name"]+".json")
        print(f"save {out_path}")
        with open(out_path, 'w') as f:
            json.dump([Cam], f, ensure_ascii=False, indent=4)

with open(config.format_camera_info_path, 'w') as f:
    json.dump(Cam_Data, f, ensure_ascii=False, indent=4)
