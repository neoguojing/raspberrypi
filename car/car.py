import gpiod
import lgpio
import time

# ---------------------------
# 树莓派 5 GPIO 芯片
# ---------------------------
GPIO_CHIP = "gpiochip4"   # 树莓派5普通GPIO几乎都在这里
LGPIO_CHIP = 0            # /dev/gpiochip0 用于 PWM（lgpio要求）


# ---------------------------
# 舵机控制类（lgpio PWM）
# ---------------------------
class Steering:
    def __init__(
        self,
        pin,
        min_angle=-45,
        max_angle=45,
        min_pulse=0.5,   # ms 对应最小角度（SG90 默认 0.5ms）
        max_pulse=2.5,   # ms 对应最大角度（SG90 默认 2.5ms）
        smoothing=True,  # 是否缓动
        step=2,          # 缓动时每次移动角度
        delay=0.01       # 缓动速度
    ):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle

        self.min_pulse = min_pulse
        self.max_pulse = max_pulse

        self.smoothing = smoothing
        self.step = step
        self.delay = delay

        self.current_angle = 0

        self.handle = lgpio.gpiochip_open(LGPIO_CHIP)
        lgpio.gpio_claim_output(self.handle, self.pin)

        # 启动时保持中位
        self.set_angle(0, smoothing=False)

    def angle_to_duty(self, angle):
        """角度 → PWM 占空比"""
        # 限制角度
        angle = max(min(angle, self.max_angle), self.min_angle)

        # 角度映射到脉宽
        span = self.max_angle - self.min_angle
        pulse = self.min_pulse + (angle - self.min_angle) / span * (self.max_pulse - self.min_pulse)

        # 转换为占空比（%）
        duty = (pulse / 20.0) * 100.0
        return duty, angle

    def set_angle(self, angle, smoothing=None):
        """设置角度（可缓动）"""
        if smoothing is None:
            smoothing = self.smoothing

        duty, angle = self.angle_to_duty(angle)

        if not smoothing:
            lgpio.tx_pwm(self.handle, self.pin, 50, duty)
            self.current_angle = angle
            time.sleep(0.03)
            return

        # ------- 缓动 -------
        step = self.step if angle > self.current_angle else -self.step
        while abs(angle - self.current_angle) > abs(step):
            self.current_angle += step
            duty_step, _ = self.angle_to_duty(self.current_angle)
            lgpio.tx_pwm(self.handle, self.pin, 50, duty_step)
            time.sleep(self.delay)

        # 到目标角
        lgpio.tx_pwm(self.handle, self.pin, 50, duty)
        self.current_angle = angle
        time.sleep(self.delay)

    def center(self):
        self.set_angle(0)

    def cleanup(self):
        lgpio.tx_pwm(self.handle, self.pin, 50, 0)  # 停止 PWM
        lgpio.gpiochip_close(self.handle)



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



# ---------------------------
# 四轮小车控制类
# ---------------------------
class FourWheelCar:
    def __init__(self, steering_pin, motor_pins_list, pwm=True):
        self.steering = Steering(steering_pin)
        self.motors = [Motor(*pins) for pins in motor_pins_list]

    def forward(self, speed=85):
        for m in self.motors:
            m.forward_run(speed)

    def backward(self, speed=80):
        for m in self.motors:
            m.backward_run(speed)

    def stop(self):
        for m in self.motors:
            m.soft_stop()

    def turn_left(self, angle=10):
        self.steering.set_angle(-abs(angle))

    def turn_right(self, angle=10):
        self.steering.set_angle(abs(angle))

    def center_steering(self):
        if self.steering:
            self.steering.center()

    def cleanup(self):
        self.stop()
        if self.steering:
            self.steering.cleanup()
        for m in self.motors:
            m.cleanup()


# ---------------------------
# 测试代码（你需要将 BOARD 改为 BCM）
# ---------------------------
if __name__ == "__main__":
    steering_pin = 18   # 示例：BCM18（原 BOARD 12）

    motor_pins = [
        (6,5),  # 前左 BOARD 29 31
        (13, 19),   # 前右 BOARD 33 35
        (16,26),    # 后左 BOARD 37 36
        (20, 21)   # 后右 BOARD 38 40
    ]

    car = FourWheelCar(steering_pin, motor_pins, pwm=True)

    try:
        while True:
            cmd = input("输入命令 w/s/a/d/q: ")
            if cmd == 'w':
                car.forward()
            elif cmd == 's':
                car.backward()
            elif cmd == 'a':
                car.turn_left(30)
            elif cmd == 'd':
                car.turn_right(30)
            elif cmd == 'q':
                car.stop()
                car.center_steering()
            else:
                print("无效命令")
    except KeyboardInterrupt:
        pass
    finally:
        car.cleanup()

