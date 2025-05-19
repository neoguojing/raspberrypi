import pyaudio

p = pyaudio.PyAudio()
info = p.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')

print("可用音频输入设备:")
for i in range(0, numdevices):
    device_info = p.get_device_info_by_host_api_device_index(0, i)
    if (device_info.get('maxInputChannels')) > 0:
        print(f"  索引 {i}: {device_info.get('name')} (输入声道数: {device_info.get('maxInputChannels')})")

print("\n可用音频输出设备:")
for i in range(0, numdevices):
    device_info = p.get_device_info_by_host_api_device_index(0, i)
    if (device_info.get('maxOutputChannels')) > 0:
        print(f"  索引 {i}: {device_info.get('name')} (输出声道数: {device_info.get('maxOutputChannels')})")

p.terminate()