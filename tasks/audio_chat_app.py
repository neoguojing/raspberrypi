
from audio.audio_play import play_audio_bytes,AudioPlayer
from audio.audio_record import ContinuousAudioListener
from clients.agi import OpenAIClient
from tools.device import check_audio_devices
from tools.utils import pcm_to_wav
from urllib.parse import urljoin
import os
import time
import asyncio
from .config import (
    POST_WAKE_COOLDOWN,
    WAKE_CHECK_SECONDS,
    WAKE_CHECK_STEP,
    WAKE_SOUND_PATH,
    AUDIO_FEATURE_TYPE,
    AGI_URL,
    AGI_API_KEY
    )
from .base import BaseTask

class VoiceAssistant(BaseTask):
    """事件驱动的唤醒检测与交互管理"""
    def __init__(self):
        super().__init__() 
        self.recorder = ContinuousAudioListener()
        self.player = AudioPlayer()
        self.client = OpenAIClient(api_key=AGI_API_KEY,base_url=AGI_URL)
        self.wake_event = asyncio.Event()
        self.cooldown = POST_WAKE_COOLDOWN or 180
        self.interaction_lock = asyncio.Lock()

    async def player_loop(self):
        url = urljoin(AGI_URL,"/audio_stream")
        await self.player.play(url)

    async def wake_checker(self):
        """
        定期从缓冲中获取音频片段，静音跳过，检测唤醒词。
        """
        while True:
            # 处于非激活状态
            if not self.wake_event.is_set():
                # 交互时无需检测
                async with self.interaction_lock:
                    data = self.recorder.get_buffered_data(WAKE_CHECK_SECONDS)
                    if self.recorder.is_silence(data):
                        await asyncio.sleep(WAKE_CHECK_STEP)
                        continue

                    path = await pcm_to_wav(data,channels=self.recorder.channels,rate=self.recorder.rate,
                                      sampwidth=self.recorder.get_sampwidth(),save_to_file=True)
                    print(path)
                    try:
                        if await self.client.audio_wakeup(path):
                            self.wake_event.set()
                    finally:
                        os.remove(path)

            await asyncio.sleep(WAKE_CHECK_STEP)

    async def single_interaction(self,clip):
        """一次录音-发送-播放交互"""
        async with self.interaction_lock:
            if clip:
                try:
                    async for content in self.client.send_audio_to_llm(clip,feature=AUDIO_FEATURE_TYPE):
                        print(content)
                finally:
                    if isinstance(clip,str):
                        os.remove(clip)

    async def _run(self):
        inputs,outpus,_,_ = await check_audio_devices()
        if not inputs or not outpus:
            raise ValueError("输入或输出设备缺失")
        # 启动监听与唤醒检测
        self.recorder.start()
        asyncio.create_task(self.wake_checker())
        asyncio.create_task(self.player_loop())

        last_interaction = 0
        print("助手已启动，持续监听中...")
        while True:
            now = time.time()
            if now - last_interaction > self.cooldown:
                # 等待唤醒事件
                self.wake_event.clear()
                await self.wake_event.wait()
                print("唤醒词检测到，准备交互...")
                if WAKE_SOUND_PATH and os.path.exists(WAKE_SOUND_PATH):
                    # await play_sound(WAKE_SOUND_PATH)
                    await play_audio_bytes(WAKE_SOUND_PATH)
                    last_interaction = time.time()
            else:
                print("冷却期内，直接交互...")
            clip = await self.recorder.record()
            if clip:
                await self.single_interaction(clip)
                last_interaction = time.time()
            

    def _stop(self):
        self.recorder.stop()
        self.player.terminate()


