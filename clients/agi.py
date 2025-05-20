import os
from openai import AsyncOpenAI
from tools.util import audio_to_base64

# 初始化异步客户端
client = AsyncOpenAI(
    api_key="123",  # 替换为你的API Key，或读取环境变量
    base_url="http://localhost:8000/v1",
)

async def audio_wakeup(audio_filepath: str, wake_up_word: str = "你好，小派") -> bool:
    """
    使用 Whisper 模型识别音频内容，并判断是否包含唤醒词。
    
    参数:
        audio_filepath (str): 音频文件路径
        wake_up_word (str): 唤醒词，默认值为“你好，小派”
    
    返回:
        bool: 是否检测到唤醒词
    """
    if not os.path.exists(audio_filepath):
        print(f"音频文件 {audio_filepath} 未找到。")
        return False

    try:
        with open(audio_filepath, 'rb') as audio_file:
            response = await client.audio.transcriptions.create(
                file=audio_file,
                model='whisper-1',
                response_format='json',
                language='zh',
            )
        print("转录结果:", response.text)
        return wake_up_word in response.text
    except Exception as e:
        print(f"音频唤醒失败: {e}")
        return False


async def send_audio_to_llm(audio_filepath: str):
    """
    将音频文件发送到远程 LLM API，并打印其响应流。
    
    参数:
        audio_filepath (str): 音频文件路径
    """
    if not os.path.exists(audio_filepath):
        print(f"音频文件 {audio_filepath} 未找到。")
        return

    try:
        audio_base64 = audio_to_base64(audio_filepath)
        stream = await client.chat.completions.create(
            model="agi-model",
            stream=True,
            extra_body={"need_speech": True, "feature": "speech"},
            messages=[
                {
                    "role": "user",
                    "content": [{"type": "audio", "audio": audio_base64}],
                }
            ],
            user="raspberrypi",
        )

        async for chunk in stream:
            print("------", chunk)
            if chunk.choices and len(chunk.choices) > 0:
                choice = chunk.choices[0]
                if choice.finish_reason is None and choice.delta:
                    print(choice.delta.content, end='', flush=True)
                    content = choice.delta.content
                    if content and len(content) > 0:
                        for item in content:
                            if item.get("type") == "audio":
                                audio = item.get("audio")
                                yield audio
                elif choice.finish_reason == "stop":
                    break
    except Exception as e:
        print(f"发送音频到 LLM 失败: {e}")
