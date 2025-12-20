import json
import pathlib
import subprocess
import config
import os

def main():
    camera_names = config.camera_names

    for cam_id in camera_names:
        # 创建输出目录
        output_dir = os.path.join(config.image_path,cam_id)
        os.makedirs(output_dir, exist_ok=True)
        # 输入视频文件
        input_video = os.path.join(config.input_video_path,f"{cam_id}.mp4")
        output_pattern = os.path.join(output_dir, "%05d.jpg")

        # ffmpeg 命令
        cmd = [
            "ffmpeg",
            "-i", input_video,
            "-q:v", str(0),
            "-vf", f"fps={1/2}",
            output_pattern
        ]

        print(f"执行: {' '.join(cmd)}")
        subprocess.run(cmd, check=False)

if __name__ == "__main__":
    main()
