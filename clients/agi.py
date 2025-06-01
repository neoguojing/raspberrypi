import os
import types
from openai import AsyncOpenAI,AsyncStream
from tools.utils import audio_to_base64
import logging
log = logging.getLogger(__name__)
class OpenAIClient:
    """
    集成 AsyncOpenAI 客户端管理和音频处理功能（唤醒词检测和发送音频到LLM）
    """

    def __init__(self, api_key=None, base_url=None):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        self.base_url = base_url or "https://api.openai.com/v1"
        self.client = AsyncOpenAI(api_key=self.api_key, base_url=self.base_url)

    async def audio_wakeup(self, audio_filepath: str, wake_up_word: str = "小派") -> bool:
        """使用 Whisper 识别音频内容，判断是否包含唤醒词"""
        if not os.path.exists(audio_filepath):
            print(f"音频文件 {audio_filepath} 未找到。")
            return False

        try:
            with open(audio_filepath, "rb") as audio_file:
                response = await self.client.audio.transcriptions.create(
                    file=audio_file,
                    model="whisper-1",
                    response_format="json",
                    language="zh",
                )
            log.info("转录结果:", response.text)
            return wake_up_word in response.text
        except Exception as e:
            print(f"音频唤醒失败: {e}")
            return False

    async def send_audio_to_llm(self, audio: any, feature: str = ""):
        """
        发送音频到 LLM，支持流式和非流式返回，异步生成Base64音频片段
        """
        try:
            audio_base64 = await audio_to_base64(audio)
            stream = (feature != "voice_chat")

            response = await self.client.chat.completions.create(
                model="agi-model",
                stream=stream,
                extra_body={"need_speech": True, "feature": feature},
                messages=[
                    {
                        "role": "user",
                        "content": [{"type": "audio", "audio": audio_base64}],
                    }
                ],
                user="raspberrypi",
            )

            async for content in self._parse_llm_response(response):
                yield content

        except Exception as e:
            log.error(f"发送音频到 LLM 失败: {e}")

    async def _parse_llm_response(self, response):
        """统一解析流式或非流式 LLM 返回"""
        if isinstance(response, (AsyncStream, types.AsyncGeneratorType)):
            async for chunk in response:
                if chunk.choices and chunk.choices[0].delta:
                    delta = chunk.choices[0].delta
                    content = delta.content
                    yield content
        else:
            choices = response.choices
            if choices and choices[0].message:
                content = choices[0].message.content
                yield content