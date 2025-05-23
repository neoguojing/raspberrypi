import argparse
import asyncio
from tasks.tasks import TASKS  # 自动收集任务函数

def parse_key_value_args(kv_args):
    kwargs = {}
    if kv_args:
        for item in kv_args:
            if "=" in item:
                k, v = item.split("=", 1)
                kwargs[k] = v
    return kwargs

def main():
    parser = argparse.ArgumentParser(description="树莓派异步任务 CLI")
    parser.add_argument("task", help="要执行的任务名", choices=TASKS.keys())
    parser.add_argument("--args", nargs="*", help="任务参数，例如 name=pi")

    args = parser.parse_args()
    kwargs = parse_key_value_args(args.args)

    task = TASKS.get(args.task)
    try:
        if task:
            asyncio.run(task.run(**kwargs))
        else:
            print(f"任务 {args.task} 未注册")
    except KeyboardInterrupt:
        print("助手终止，清理...")
        task.stop()
        
