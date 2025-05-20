import pyaudio
import asyncio

async def check_audio_devices(expected_input=None, expected_output=None):
    """
    检查是否存在指定名称的音频输入和输出设备。

    参数:
        expected_input (str): 期望的输入设备名称（部分匹配）
        expected_output (str): 期望的输出设备名称（部分匹配）

    返回:
        dict: 包含所有输入设备、输出设备及是否找到目标设备的信息
    """
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    input_devices = []
    output_devices = []
    input_found = False
    output_found = False

    for i in range(numdevices):
        device_info = p.get_device_info_by_host_api_device_index(0, i)
        name = device_info.get('name', '')
        max_input = device_info.get('maxInputChannels', 0)
        max_output = device_info.get('maxOutputChannels', 0)

        if max_input > 0:
            input_devices.append({'index': i, 'name': name, 'channels': max_input})
            if expected_input and expected_input.lower() in name.lower():
                input_found = True

        if max_output > 0:
            output_devices.append({'index': i, 'name': name, 'channels': max_output})
            if expected_output and expected_output.lower() in name.lower():
                output_found = True

    p.terminate()

    return {
        'input_devices': input_devices,
        'output_devices': output_devices,
        'input_device_found': input_found if expected_input else None,
        'output_device_found': output_found if expected_output else None
    }

# 示例调用
async def main():
    result = await check_audio_devices(expected_input="USB", expected_output="HDMI")
    print("输入设备列表:")
    for d in result['input_devices']:
        print(f"  索引 {d['index']}: {d['name']} (声道数: {d['channels']})")

    print("\n输出设备列表:")
    for d in result['output_devices']:
        print(f"  索引 {d['index']}: {d['name']} (声道数: {d['channels']})")

    if result['input_device_found'] is not None:
        print(f"\n是否找到指定输入设备: {'是' if result['input_device_found'] else '否'}")

    if result['output_device_found'] is not None:
        print(f"是否找到指定输出设备: {'是' if result['output_device_found'] else '否'}")

# 运行异步函数
if __name__ == "__main__":
    asyncio.run(main())
