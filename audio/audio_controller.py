import asyncio
from .audio_play import AudioPlayer
from .audio_record import ContinuousAudioListener
import logging
log = logging.getLogger(__name__)
class AudioControllerAsync:
    def __init__(self):
        self.recorder = ContinuousAudioListener()
        self.recorder.start()
        self.player = AudioPlayer()
    
    async def run(self,url: str):
        asyncio.gather(self.player.play(),self.player.producer(url))
        
    async def record(self):
        # 不用锁，改为检查播放队列状态
        wait_count = 0
        while not self.player.queue.empty():
            await asyncio.sleep(0.1)
            wait_count += 1
            if wait_count > 100:
                log.warning("播放队列长时间不为空，放弃录音")
                return None

        log.info("播放队列为空，开始录音")
        clip = await self.recorder.record()
        log.info("录音结束")
        return clip
    
    def get_buffered_data(self, seconds: int):
        return self.recorder.get_buffered_data(seconds)
        
    def get_recorder_param(self):
        return self.recorder.channels,self.recorder.rate,self.recorder.get_sampwidth()
        
    def is_silence(self,frame: bytes):
        return self.recorder.is_silence(frame)
        
    def terminate(self):
        self.recorder.stop()
        self.player.terminate()
            
