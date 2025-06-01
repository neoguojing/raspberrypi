import asyncio
from .audio_play import AudioPlayer
from .audio_record import ContinuousAudioListener
import logging
log = logging.getLogger(__name__)
class AudioControllerAsync:
    def __init__(self):
        self.lock = asyncio.Lock()
        self.recorder = ContinuousAudioListener()
        self.recorder.start()
        self.player = AudioPlayer()
    
    async def run(self):
        asyncio.create_task(self.player.play())
        
    async def record(self):
        async with self.lock:
            log.info("开始录音")
            clip = await self.recorder.record()
            log.info("结束录音")
            return clip

    async def play(self,url):
        async with self.lock:
            log.info("开始播放")
            await self.player.producer(url)
            log.info("播放结束")
    
    def get_buffered_data(self, seconds: int):
        return self.recorder.get_buffered_data(seconds)
        
    def get_recorder_param(self):
        return self.recorder.channels,self.recorder.rate,self.recorder.get_sampwidth()
        
    def is_silence(self,frame: bytes):
        return self.recorder.is_silence(frame)
        
    def terminate(self):
        self.recorder.stop()
        self.player.terminate()
            
