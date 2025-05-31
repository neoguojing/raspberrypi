import asyncio
from .audio_play import AudioPlayer
from .audio_record import ContinuousAudioListener
class AudioControllerAsync:
    def __init__(self):
        self.lock = asyncio.Lock()
        self.recorder = ContinuousAudioListener()
        self.recorder.start()
        self.player = AudioPlayer()

    async def record(self):
        async with self.lock:
            print("开始录音")
            clip = await self.recorder.record()
            print("录音结束")
            return clip

    async def play(self,url):
        async with self.lock:
            print("开始播放")
            await self.player.play(url)
            print("播放结束")
            
    def terminate(self):
        self.recorder.stop()
        self.player.terminate()
            
