from audio.audio_play import play_sound
from audio.audio_record import record_audio
from clients.agi import audio_wakeup,send_audio_to_llm
TASKS = {
    "chat": play_sound,
}