import json
import pathlib
import config
import os

GRID_SIZE = (2.88, 2.0)
MAP_SIZE = (28.87, 19.11) #单位英尺

def grid_2_xy(a, b):
    assert b <= 10 and b >= 0
    gw = GRID_SIZE[0]
    gh = GRID_SIZE[1]
    if b < 1:
        return (gw * a, 1.83 * b)
    elif b <= 9:
        return (gw * a, gh * (b - 1) + 1.83)
    else :
        return (gw * a, gh * (b - 2) + 1.83 + 1.26 * (b - 9))

if __name__ == '__main__':

    camera_ids = config.camera_ids
    camera_names = config.camera_names
    camera_types = config.camera_types or ["单目"] * len(camera_ids)
    camera_resolution = config.camera_resolution
    print(camera_ids)
    print(camera_names)
    print(camera_resolution)
    data = []
    for i in range(len(camera_names)):
        if camera_ids == None:
            id = i + 1
        else:
            id = camera_ids[i]
        
        name = camera_names[i]
        
        if camera_types == None:
            type = '单目'
        else:
            type = camera_types[i]
        
        resolution = camera_resolution[i]

        dict = {}
        dict['id'] = id
        dict['name'] = name
        dict['type'] = type
        dict['resolution'] = resolution

        data.append(dict)

    # === 新增逻辑：若文件存在，只更新变动字段，不覆盖其他 key ===
    if os.path.exists(config.camera_info_path):
        try:
            with open(config.camera_info_path, 'r', encoding='utf-8') as f:
                old_data = json.load(f)

            # 构建 {id -> 原记录} 映射
            old_map = {item.get('id'): item for item in old_data if 'id' in item}

            for item in data:
                cid = item['id']
                if cid in old_map:
                    # 仅更新已知字段，不覆盖其他 key
                    for key in ['name', 'type', 'resolution']:
                        old_value = old_map[cid].get(key)
                        new_value = item[key]
                        if old_value != new_value:
                            old_map[cid][key] = new_value
                            print(f"更新相机 {cid} 字段 {key}: {old_value} -> {new_value}")
                else:
                    # 新增相机，直接加入
                    old_map[cid] = item
                    print(f"新增相机 {cid}")

            # 写回时保留其他字段
            new_data = list(old_map.values())

            with open(config.camera_info_path, 'w', encoding='utf-8') as f:
                json.dump(new_data, f, ensure_ascii=False, indent=4)
            print("配置文件已部分更新（保留其他字段）。")

        except Exception as e:
            print(f"加载旧配置失败，重新生成：{e}")
            with open(config.camera_info_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=4)
    else:
        with open(config.camera_info_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
        print("首次生成配置文件。")

