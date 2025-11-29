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
    def __init__(self, pin, min_angle=-45, max_angle=45):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle

        self.handle = lgpio.gpiochip_open(LGPIO_CHIP)
        lgpio.gpio_claim_output(self.handle, self.pin)

        # 初始中位
        self.set_angle(0)
        time.sleep(0.2)

    def set_angle(self, angle):
        angle = max(min(angle, self.max_angle), self.min_angle)

        # 计算舵机 PWM 占空比
        # 0.5ms - 2.5ms 之间 (2ms 范围)
        duty_ms = 1.5 + (angle / 90.0)  # -45~45 对应 ±0.5ms
        duty_cycle = duty_ms / 20.0 * 100  # 转成占空比

        lgpio.tx_pwm(self.handle, self.pin, 50, duty_cycle)
        time.sleep(0.1)

    def center(self):
        self.set_angle(0)

    def cleanup(self):
        lgpio.tx_pwm(self.handle, self.pin, 50, 0)
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

    # -------------------------
    # 内部：设置 PWM
    # -------------------------
    def _apply_pwm(self, bwd_duty,fwd_duty):
        """内部方法，确保不会正反同时输出"""

        fwd_duty = int(max(0, min(100, fwd_duty)))
        bwd_duty = int(max(0, min(100, bwd_duty)))
        print(fwd_duty,bwd_duty)

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
    def backward_run(self, speed=10):
        self._apply_pwm(speed, 0)
        self.current_speed = -speed

    # -------------------------
    # 电机正转（前进）
    # -------------------------
    def forward_run(self, speed=15):
        self._apply_pwm(0, speed)
        self.current_speed = speed

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
        # self.steering = Steering(steering_pin)
        self.motors = [Motor(*pins) for pins in motor_pins_list]

    def forward(self, speed=100):
        for m in self.motors:
            m.forward_run(speed)

    def backward(self, speed=100):
        for m in self.motors:
            m.backward_run(speed)

    def stop(self):
        for m in self.motors:
            m.soft_stop()

    def turn_left(self, angle=30):
        self.steering.set_angle(-abs(angle))

    def turn_right(self, angle=30):
        self.steering.set_angle(abs(angle))

    def center_steering(self):
        self.steering.center()

    def cleanup(self):
        self.stop()
        self.steering.cleanup()
        for m in self.motors:
            m.cleanup()


# ---------------------------
# 测试代码（你需要将 BOARD 改为 BCM）
# ---------------------------
if __name__ == "__main__":
    steering_pin = 18   # 示例：BCM18（原 BOARD 12）

    motor_pins = [
        (5, 6),  # 前左 BOARD 29 31
        (13, 19),   # 前右 BOARD 33 35
        (26, 16),    # 后左 BOARD 37 36
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

