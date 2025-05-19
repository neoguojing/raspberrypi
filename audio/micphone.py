# main.py
import os
import struct
import wave
import time
import pvporcupine
import pyaudio
import requests
import audioop # 用于RMS计算

import audio.config as config

# --- 全局变量 ---
porcupine = None
pa = None
audio_stream = None

def record_audio(output_filename="recorded_audio.wav"):
    """
    监听到唤醒词后录制音频，直到静音或超时。
    """
    global audio_stream, pa

    if audio_stream is None or pa is None:
        pa = pyaudio.PyAudio()
        try:
            audio_stream = pa.open(
                rate=config.AUDIO_RATE,
                channels=config.AUDIO_CHANNELS,
                format=pyaudio.paInt16, # 16-bit 采样
                input=True,
                frames_per_buffer=config.AUDIO_CHUNK_SIZE,
                input_device_index=config.AUDIO_DEVICE_INDEX
            )
        except Exception as e:
            print(f"无法打开麦克风流: {e}")
            print("请检查config.py中的AUDIO_DEVICE_INDEX是否正确。")
            print("可运行 list_audio_devices.py (辅助脚本) 查看可用设备。")
            if pa: pa.terminate()
            pa = None
            audio_stream = None
            return None

    print("开始录音...")
    if os.path.exists(config.WAKE_SOUND_PATH):
        play_sound(config.WAKE_SOUND_PATH)
    else: # 简单的嘟声作为替代
        # 在Linux上，你可以使用 `os.system('paplay /usr/share/sounds/freedesktop/stereo/dialog-information.oga &')`
        # 或者使用pygame生成一个简单的音调
        print("* recording start beep *")


    frames = []
    silent_chunks = 0
    recording_started_time = time.time()

    for _ in range(0, int(config.AUDIO_RATE / config.AUDIO_CHUNK_SIZE * config.RECORD_SECONDS_AFTER_WAKE)):
        try:
            data = audio_stream.read(config.AUDIO_CHUNK_SIZE, exception_on_overflow=False)
            frames.append(data)

            # 简单的静音检测 (基于RMS)
            rms = audioop.rms(data, 2)  # 2 is for signed 16-bit samples
            # print(f"RMS: {rms}") # 调试时打开
            if rms < config.SILENCE_THRESHOLD:
                silent_chunks += 1
            else:
                silent_chunks = 0

            if silent_chunks > config.SILENT_CHUNKS_NEEDED:
                print("检测到静音，停止录音。")
                break
        except IOError as ex:
            if ex.errno == pyaudio.paInputOverflowed:
                print("输入溢出，忽略...")
                continue # 或者做其他处理
            else:
                raise
        except Exception as e:
            print(f"录音时发生错误: {e}")
            break
            
    print("录音结束。")
    if os.path.exists(config.END_SOUND_PATH):
        play_sound(config.END_SOUND_PATH)
    else:
        print("* recording end beep *")


    # 停止并关闭流，但不终止PyAudio对象，以便下次复用
    # audio_stream.stop_stream()
    # audio_stream.close()
    # pa.terminate() # 如果每次都重新初始化PyAudio，则需要调用

    # 保存录音文件
    wf = wave.open(output_filename, 'wb')
    wf.setnchannels(config.AUDIO_CHANNELS)
    wf.setsampwidth(pa.get_sample_size(pyaudio.paInt16))
    wf.setframerate(config.AUDIO_RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    return output_filename



def main():
    global porcupine, pa, audio_stream
    try:
        porcupine = pvporcupine.create(
            access_key=config.PICOVOICE_ACCESS_KEY,
            library_path=config.PORCUPINE_LIBRARY_PATH,
            model_path=config.PORCUPINE_MODEL_PATH,
            keyword_paths=config.PORCUPINE_KEYWORD_PATHS,
            sensitivities=config.PORCUPINE_SENSITIVITIES
        )

        pa = pyaudio.PyAudio()
        # 为Porcupine打开音频流
        porcupine_audio_stream = pa.open(
            rate=porcupine.sample_rate,
            channels=1, # Porcupine 需要单声道
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=porcupine.frame_length,
            input_device_index=config.AUDIO_DEVICE_INDEX # 使用与录音相同的设备
        )
        print(f"使用的麦克风采样率: {porcupine.sample_rate}, 帧长度: {porcupine.frame_length}")
        print(f"正在监听唤醒词: {config.PORCUPINE_KEYWORD_PATHS}...")
        print("请说出唤醒词 (例如 'Raspberry Pi')... 按 Ctrl+C 退出")

        while True:
            try:
                pcm = porcupine_audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
                pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)
            except IOError as ex:
                if ex.errno == pyaudio.paInputOverflowed:
                    print("Porcupine 输入溢出，忽略...")
                    continue
                else:
                    raise
            except Exception as e:
                print(f"读取Porcupine音频流错误: {e}")
                time.sleep(0.1) # 避免快速循环错误
                continue


            keyword_index = porcupine.process(pcm)

            if keyword_index >= 0:
                print(f"唤醒词 '{os.path.basename(config.PORCUPINE_KEYWORD_PATHS[keyword_index]).split('_')[0]}' 被检测到!")
                
                # 唤醒后，关闭Porcupine的流，准备录制主音频
                # （或者，你可以让Porcupine流继续，然后从同一个流中截取数据给主录音逻辑，
                #  但这会使主录音逻辑依赖Porcupine的帧长度和采样率，如果它们不同就需要重采样）
                # 为简单起见，这里我们先关闭，再为录音打开新的流（或复用已关闭的audio_stream）
                # 如果 audio_stream 已经在 record_audio 中被正确管理（打开和关闭），这里可能不需要特殊操作
                # porcupine_audio_stream.stop_stream() # 临时停止，以便录音模块使用麦克风

                recorded_file = record_audio() # audio_stream 会在 record_audio 中被创建或复用

                # porcupine_audio_stream.start_stream() # 录音结束后，重新启动Porcupine的流

                if recorded_file:
                    llm_response_text = send_audio_to_llm(recorded_file)
                    if llm_response_text:
                        speak_text(llm_response_text)
                    else:
                        speak_text("抱歉，我没有从服务器获取到有效的回答。")
                    
                    try:
                        os.remove(recorded_file) # 清理录音文件
                    except OSError as e:
                        print(f"删除 {recorded_file} 失败: {e}")
                else:
                    speak_text("抱歉，录音失败了。")
                
                print(f"\n再次监听唤醒词: {config.PORCUPINE_KEYWORD_PATHS}...")


    except pvporcupine.PorcupineError as e:
        print(f"Porcupine 错误: {e}")
        print("请检查: ")
        print("1. PICOVOICE_ACCESS_KEY 是否正确。")
        print("2. 模型文件和关键词文件路径是否正确，且与你的操作系统/架构匹配。")
        print("3. 树莓派是否联网 (首次运行时可能需要验证AccessKey)。")
    except KeyboardInterrupt:
        print("\n用户中断，程序退出。")
    except Exception as e:
        print(f"发生未预料的错误: {e}")
    finally:
        print("清理资源...")
        if audio_stream is not None and audio_stream.is_active():
            audio_stream.stop_stream()
            audio_stream.close()
        
        if 'porcupine_audio_stream' in locals() and porcupine_audio_stream is not None and porcupine_audio_stream.is_active():
            porcupine_audio_stream.stop_stream()
            porcupine_audio_stream.close()

        if pa is not None:
            pa.terminate()

        if porcupine is not None:
            porcupine.delete()
        
        mixer.quit()
