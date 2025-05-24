#!/bin/bash

# echo "✅ 设置 SDL 使用 PulseAudio..."
# export SDL_AUDIODRIVER=pulse
# export PULSE_SERVER=localhost
# export PULSE_SERVER=/run/user/1000/pulse/native
# export SDL_AUDIODRIVER=alsa


# echo "✅ 配置 ALSA 使用 USB 麦克风（card 2, device 0）..."
# cat > ~/.asoundrc <<EOF
# # 设置默认的音频输入设备（USB 麦克风）
# # 设置默认的音频输入设备（USB 麦克风）
# pcm.!default {
#     type hw
#     card 2        # USB 麦克风的音频卡
#     device 0      # 麦克风的设备编号
# }

# ctl.!default {
#     type hw
#     card 2        # 控制接口使用 USB 麦克风的卡
# }

# # 蓝牙音响配置（PulseAudio）
# pcm.bluetooth {
#     type pulse
#     device bluez_sink.58_EA_1F_26_88_47.a2dp_sink
# }

# ctl.bluetooth {
#     type pulse
#     device bluez_sink.58_EA_1F_26_88_47.a2dp_sink
# }
# EOF

# cat > ~/.asoundrc <<EOF
# pcm.!default {
#     type asym
#     capture.pcm "mic"
#     playback.pcm "bluetooth"
# }

# pcm.mic {
#     type hw
#     card 2
#     device 0
# }

# pcm.bluetooth {
#     type bluealsa
#     device "58:EA:1F:26:88:47"  # 替换为你的蓝牙音响设备的 MAC 地址
#     profile "a2dp"
# }
# EOF

# echo "✅ 设置 PulseAudio 默认输入和输出设备..."

# # 设置默认输入为 USB 麦克风
# pactl set-default-source alsa_input.usb-C-Media_Electronics_Inc._USB_PnP_Sound_Device-00.analog-mono

# # 设置默认输出为 蓝牙音响
# pactl set-default-sink bluez_sink.58_EA_1F_26_88_47.a2dp_sink

# echo "✅ 当前输入设备："
# pactl info | grep "Default Source"

# echo "✅ 当前输出设备："
# pactl info | grep "Default Sink"

# echo "🚀 启动应用..."
python cli.py task chat # 请替换为你的游戏脚本文件名
