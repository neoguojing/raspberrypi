# world 模块说明

`world/` 提供 Gazebo 场景文件（`.sdf`），用于机器人仿真测试。

## 场景文件

- `bed_room.sdf`
- `bed_room_origin.sdf`
- `tugbot_depot.sdf`

## 使用示例

```bash
gz sim -v 4 world/bed_room.sdf
```

如需完整仿真依赖安装，请参考 `robot/README`。
