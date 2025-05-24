class BaseTask:
    def __init__(self):
        self._is_running = False  # 用于追踪任务是否正在运行

    # 保护的异步方法：用于执行任务
    async def _run(self):
        """子类应重写此方法实现具体任务"""
        raise NotImplementedError("Subclasses should implement the '_run' method.")

    # 保护的同步方法：停止任务
    def _stop(self):
        """子类应重写此方法实现停止逻辑"""
        raise NotImplementedError("Subclasses should implement the '_stop' method.")
    
    # 对外公开的接口，启动任务
    async def run(self):
        """启动任务并保证只能运行一次"""
        if self._is_running:
            print("Task is already running.")
            return
        self._is_running = True
        print("Task started.")
        await self._run()
    
    # 对外公开的接口，停止任务
    def stop(self):
        """停止任务"""
        if not self._is_running:
            print("Task is not running.")
            return
        self._is_running = False
        self._stop()
        print("Task stopped.")