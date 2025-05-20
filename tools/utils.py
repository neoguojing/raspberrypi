import base64
def audio_to_base64(audio_path):
    with open(audio_path, "rb") as audio_file:
        encoded_audio = base64.b64encode(audio_file.read()).decode('utf-8')
    return encoded_audio