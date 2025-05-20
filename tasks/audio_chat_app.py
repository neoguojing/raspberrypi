
from audio.audio_play import play_sound
from audio.audio_record import ContinuousAudioListener
from clients.agi import audio_wakeup,send_audio_to_llm
import os
import time
import asyncio
import wave
import tempfile
import audioop
import config


class VoiceAssistant:
    """事件驱动的唤醒检测与交互管理"""
    def __init__(self):
        self.listener = ContinuousAudioListener()
        self.wake_event = asyncio.Event()
        self.cooldown = getattr(config, 'POST_WAKE_COOLDOWN', 30)
        self.interaction_lock = asyncio.Lock()

    async def wake_checker(self):
        """
        定期从缓冲中获取音频片段，静音跳过，检测唤醒词。
        """
        while True:
            data = self.listener.get_buffered_data(config.WAKE_CHECK_SECONDS)
            if audioop.rms(data, 2) < config.SILENCE_THRESHOLD:
                await asyncio.sleep(config.WAKE_CHECK_STEP)
                continue

            # 写入临时文件并检测唤醒
            path = os.path.join(tempfile.gettempdir(), f"wake_{int(time.time())}.wav")
            with wave.open(path, 'wb') as wf:
                wf.setnchannels(self.listener.channels)
                wf.setsampwidth(self.listener.pa.get_sample_size(pyaudio.paInt16))
                wf.setframerate(self.listener.rate)
                wf.writeframes(data)

            try:
                if await audio_wakeup(path):
                    self.wake_event.set()
            finally:
                os.remove(path)

            await asyncio.sleep(config.WAKE_CHECK_STEP)

    async def single_interaction(self):
        """一次录音-发送-播放交互"""
        async with self.interaction_lock:
            clip = await self.listener.record()
            try:
                resp = await send_audio_to_llm(clip)
                if resp and os.path.exists(resp):
                    await play_sound(resp)
                    os.remove(resp)
            finally:
                os.remove(clip)

    async def run(self):
        # 启动监听与唤醒检测
        self.listener.start()
        asyncio.create_task(self.wake_checker())

        print("助手已启动，持续监听中...")
        last_interaction = 0

        while True:
            await self.wake_event.wait()
            now = time.time()
            if now - last_interaction >= self.cooldown:
                print("唤醒词检测到，准备交互...")
                if config.WAKE_SOUND_PATH and os.path.exists(config.WAKE_SOUND_PATH):
                    await play_sound(config.WAKE_SOUND_PATH)
            else:
                print("冷却期内，直接交互...")

            await self.single_interaction()
            last_interaction = time.time()
            self.wake_event.clear()

    def stop(self):
        self.listener.stop()


if __name__ == "__main__":
    va = VoiceAssistant()
    try:
        asyncio.run(va.run())
    except KeyboardInterrupt:
        print("助手终止，清理...")
        va.stop()
