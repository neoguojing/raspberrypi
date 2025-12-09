import lgpio
import time
# ---------------------------
# 树莓派 5 GPIO 芯片
# ---------------------------
LGPIO_CHIP = 0            # /dev/gpiochip0 用于 PWM（lgpio要求）



class Motor:
    """
    N20 电机优化版本
    使用:
      - forward_pin: H 桥正转输入
      - backward_pin: H 桥反转输入
      - freq: PWM频率，推荐 10kHz
    """

    def __init__(self, forward_pin, backward_pin, freq=10000):
        self.forward = forward_pin
        self.backward = backward_pin
        self.freq = freq

        # 树莓派5使用 lgpio 做高性能PWM
        self.handle = lgpio.gpiochip_open(LGPIO_CHIP)

        lgpio.gpio_claim_output(self.handle, self.forward)
        lgpio.gpio_claim_output(self.handle, self.backward)

        # 初始电平归零
        lgpio.gpio_write(self.handle, self.forward, 0)
        lgpio.gpio_write(self.handle, self.backward, 0)

        # 当前速度缓存（可用于渐变加速）
        self.current_speed = 0
        # 当前方向：0 表示前进，1表示后退
        self.last_dir = 0

    # -------------------------
    # 内部：设置 PWM
    # -------------------------
    def _apply_pwm(self, fwd_duty,bwd_duty):
        """内部方法，确保不会正反同时输出"""

        fwd_duty = int(max(0, min(100, fwd_duty)))
        bwd_duty = int(max(0, min(100, bwd_duty)))
        print(fwd_duty,bwd_duty)

        if fwd_duty > 0 and bwd_duty > 0:
            # 可以选择报错，或自动处理（比如优先正转，或置零）
            # 这里建议抛出异常，防止逻辑错误
            raise ValueError("Cannot apply forward and backward PWM simultaneously!")

        if fwd_duty > 0:
            lgpio.tx_pwm(self.handle, self.forward, self.freq, fwd_duty)
        else:
            lgpio.tx_pwm(self.handle, self.forward, self.freq, 0)

        if bwd_duty > 0:
            lgpio.tx_pwm(self.handle, self.backward, self.freq, bwd_duty)
        else:
            lgpio.tx_pwm(self.handle, self.backward, self.freq, 0)

    # -------------------------
    # 电机反转（后退）
    # -------------------------
    def backward_run(self, speed=60):
        if self.last_dir == 0:
            self.stop()
            time.sleep(0.05)
        self._apply_pwm(0, speed)
        self.current_speed = -speed
        self.last_dir = 1

    # -------------------------
    # 电机正转（前进）
    # -------------------------
    def forward_run(self, speed=85):
        if self.last_dir == 1:
            self.stop()
            time.sleep(0.05)
        self._apply_pwm(speed, 0)
        self.current_speed = speed
        self.last_dir = 0

    # -------------------------
    # 停止（硬刹车）
    # -------------------------
    def stop(self):
        self._apply_pwm(0, 0)
        self.current_speed = 0

    # -------------------------
    # 软刹车（减速到停）
    # -------------------------
    def soft_stop(self, step=5, delay=0.02):
        if self.current_speed == 0:
            return

        target = 0
        step = abs(step)
        speed = abs(self.current_speed)

        # 逐步减速
        while speed > 0:
            speed -= step
            speed = max(0, speed)

            if self.current_speed > 0:
                self.forward_run(speed)
            else:
                self.backward_run(speed)

            time.sleep(delay)

        self.stop()

    # -------------------------
    # 清理
    # -------------------------
    def cleanup(self):
        self.stop()
        lgpio.gpiochip_close(self.handle)
