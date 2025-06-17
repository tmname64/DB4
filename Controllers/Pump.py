from machine import Pin, PWM
import utime

class PumpMotor:
    def __init__(self, pinDirection, pinStep, interval = 50) -> None:
        self.A = Pin(pinDirection, Pin.OUT)
        self.B = Pin(pinStep, Pin.OUT)
        self.interval = interval

    def oneStepForward(self):
        self.A.value(1)
        self.B.value(0)
        utime.sleep_ms(self.interval)
        self.A.value(0)
        self.B.value(0)
        utime.sleep_ms(self.interval)

    def oneStepBackward(self):
        self.A.value(0)
        self.B.value(1)
        utime.sleep_ms(self.interval)
        self.A.value(0)
        self.B.value(0)
        utime.sleep_ms(self.interval)

    def moveForward(self, steps):
        print("Moving Pump Motor - Forward")
        for _ in range(steps):
            self.oneStepForward()

    def moveBackward(self, steps):
        print("Moving Pump Motor - Backward")
        for _ in range(steps):
            self.oneStepBackward()

    
