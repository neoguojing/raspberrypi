import os
import types
from openai import AsyncOpenAI,AsyncStream
from tools.utils import audio_to_base64

# 初始化异步客户端
client = AsyncOpenAI(
    api_key="123",  # 替换为你的API Key，或读取环境变量
    base_url="http://192.168.1.123:8000/v1",
)

async def audio_wakeup(audio_filepath: str, wake_up_word: str = "小派") -> bool:
    """
    使用 Whisper 模型识别音频内容，并判断是否包含唤醒词。
    
    参数:
        audio_filepath (str): 音频文件路径
        wake_up_word (str): 唤醒词，默认值为“小派”
    
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


async def send_audio_to_llm(audio: any,feature=""):
    """
    将音频文件发送到远程 LLM API，并打印其响应流。
    
    参数:
        audio (any): 音频
    """
    stream = False
    if feature != "voice_chat":
        stream = True
    try:
        audio_base64 = await audio_to_base64(audio)
        ret = await client.chat.completions.create(
            model="agi-model",
            stream=stream,
            extra_body={"need_speech": True,"feature":feature},
            messages=[
                {
                    "role": "user",
                    "content": [{"type": "audio", "audio": audio_base64}],
                }
            ],
            user="raspberrypi",
        )
        print("------", ret,type(ret))
        
        if isinstance(ret, types.GeneratorType) or isinstance(ret, AsyncStream) :
            async for chunk in ret:
                print("------", chunk)
                if chunk.choices and len(chunk.choices) > 0:
                    choice = chunk.choices[0]
                    if choice.finish_reason is None and choice.delta:
                        print(choice.delta.content, end='', flush=True)
                        content = choice.delta.content
                        if content and isinstance(content,list) and len(content) > 0:
                            for item in content:
                                if item.get("type") == "audio":
                                    audio = item.get("audio")
                                    yield audio
                    elif choice.finish_reason == "stop":
                        break
        else:
            if ret.choices and len(ret.choices) > 0:
                if ret.choices[0].message:
                    if isinstance(ret.choices[0].message.content,list):
                        content = ret.choices[0].message.content
                        if content and len(content) > 0:
                            for item in content:
                                if item.get("type") == "audio":
                                    audio = item.get("audio")
                                    yield audio
                    else:
                        print(ret.choices[0].message.content)
                    
    except Exception as e:
        print(f"发送音频到 LLM 失败: {e}")
