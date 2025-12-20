# import pigpio
# import time
# class SteeringWithpig:
#     def __init__(
#         self,
#         pin,
#         min_angle=-45,
#         max_angle=45,
#         min_pulse_us=500,   # SG90 最小脉宽 (微秒)
#         max_pulse_us=2500,  # SG90 最大脉宽 (微秒)
#         mid_offset_us=+10,  # 中位校准偏移（单位：微秒）
#         smoothing=True,
#         step=2,
#         delay=0.01
#     ):
#         self.pin = pin
#         self.min_angle = min_angle
#         self.max_angle = max_angle
#         self.min_pulse_us = min_pulse_us
#         self.max_pulse_us = max_pulse_us
#         self.mid_offset_us = mid_offset_us
#         self.smoothing = smoothing
#         self.delay = delay
#         self.current_angle = 0

#         self.pi = pigpio.pi()
#         if not self.pi.connected:
#             raise RuntimeError("无法连接到 pigpio 守护进程，请确保已运行 'sudo pigpiod'")

#         # 初始化到中位（不缓动）
#         self.set_angle(0, smoothing=False)

#     def angle_to_pulse_us(self, angle):
#         """角度 → 脉宽（微秒），带中位校准"""
#         angle = max(min(angle, self.max_angle), self.min_angle)
#         span = self.max_angle - self.min_angle
#         if span == 0:
#             pulse_us = (self.min_pulse_us + self.max_pulse_us) // 2
#         else:
#             pulse_us = self.min_pulse_us + (angle - self.min_angle) / span * (self.max_pulse_us - self.min_pulse_us)
        
#         pulse_us += self.mid_offset_us
#         pulse_us = int(max(500, min(2500, pulse_us)))  # SG90 安全范围
#         return pulse_us, angle

#     def set_angle(self, angle, smoothing=None):
#         if smoothing is None:
#             smoothing = self.smoothing

#         target_pulse, target_angle = self.angle_to_pulse_us(angle)

#         if not smoothing:
#             self.pi.set_servo_pulsewidth(self.pin, target_pulse)
#             self.current_angle = target_angle
#             return

#         # --- 缓动逻辑 ---
#         start_pulse, _ = self.angle_to_pulse_us(self.current_angle)
#         diff = target_pulse - start_pulse
#         steps = max(1, abs(diff) // 10)  # 每步约 10 微秒

#         for i in range(1, steps + 1):
#             pulse = start_pulse + diff * i // steps
#             self.pi.set_servo_pulsewidth(self.pin, pulse)
#             time.sleep(self.delay)

#         self.pi.set_servo_pulsewidth(self.pin, target_pulse)
#         self.current_angle = target_angle

#     def center(self):
#         self.set_angle(0)

#     def cleanup(self):
#         self.pi.set_servo_pulsewidth(self.pin, 0)  # 停止 PWM
#         self.pi.stop()

# ---------------------------
# 舵机控制类（lgpio PWM）
# ---------------------------
import lgpio
import time

LGPIO_CHIP = 0

class Steering:
    def __init__(
        self,
        pin,
        min_angle=-45,
        max_angle=45,
        min_pulse_ms=0.5,   # SG90 最小脉宽 (ms)
        max_pulse_ms=2.5,   # SG90 最大脉宽 (ms)
        mid_offset_us=0,  # 中位校准偏移（关键！单位：微秒）
        smoothing=True,
        step=2,
        delay=0.01
    ):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pulse_ms = min_pulse_ms
        self.max_pulse_ms = max_pulse_ms
        self.mid_offset_us = mid_offset_us  # 微秒偏移
        self.smoothing = smoothing
        self.step = step
        self.delay = delay
        self.current_angle = 0

        self.handle = lgpio.gpiochip_open(LGPIO_CHIP)
        lgpio.gpio_claim_output(self.handle, self.pin)

        # 初始化到中位（不缓动）
        self.set_angle(0, smoothing=False)

    def angle_to_duty_percent(self, angle):
        """角度 → 占空比（%），带中位校准"""
        # 限制角度范围
        angle = max(min(angle, self.max_angle), self.min_angle)
        
        # 映射到脉宽（毫秒）
        span = self.max_angle - self.min_angle
        if span == 0:
            pulse_ms = (self.min_pulse_ms + self.max_pulse_ms) / 2.0
        else:
            pulse_ms = self.min_pulse_ms + (angle - self.min_angle) / span * (self.max_pulse_ms - self.min_pulse_ms)
        
        # 应用中位偏移（微秒 → 毫秒）
        pulse_ms += self.mid_offset_us / 1000.0
        
        # 限制安全范围（SG90 通常 0.5~2.5ms）
        pulse_ms = max(0.5, min(2.5, pulse_ms))
        
        # 转换为占空比（%）：周期 = 20ms (50Hz)
        duty_percent = (pulse_ms / 20.0) * 100.0
        return duty_percent, angle

    def set_angle(self, angle, smoothing=None):
        if smoothing is None:
            smoothing = self.smoothing

        target_duty, target_angle = self.angle_to_duty_percent(angle)

        if not smoothing:
            # 直接设置 PWM（无限循环）
            lgpio.tx_pwm(self.handle, self.pin, 50, target_duty)
            self.current_angle = target_angle
            return

        # --- 缓动逻辑 ---
        start_duty, _ = self.angle_to_duty_percent(self.current_angle)
        diff = target_duty - start_duty
        steps = max(1, int(abs(diff) / 0.2))  # 每步约 0.2%

        for i in range(1, steps + 1):
            duty = start_duty + diff * i / steps
            lgpio.tx_pwm(self.handle, self.pin, 50, duty)
            time.sleep(self.delay)
        
        lgpio.tx_pwm(self.handle, self.pin, 50, target_duty)
        self.current_angle = target_angle

    def center(self):
        self.set_angle(0)

    def cleanup(self):
        # 停止 PWM：设频率为 0 或占空比为 0
        lgpio.tx_pwm(self.handle, self.pin, 0, 0)  # 频率为0表示关闭
        lgpio.gpiochip_close(self.handle)
