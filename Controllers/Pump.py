from machine import Pin, PWM
import utime
import time

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
            time.sleep(1)

    def moveBackward(self, steps):
        print("Moving Pump Motor - Backward")
        for _ in range(steps):
            self.oneStepBackward()
            time.sleep(1)


class Pump:
    def __init__(self, pinDirection, pinStep, speed) -> None:
        if speed >= 0 and speed <= 100:
            self.direction = Pin(pinDirection, Pin.OUT)
            self.direction.value(0)
            self.step = PWM(Pin(pinStep), freq = 1000, duty = int(1023/100*speed))
        else:
            print("Dalban, set speed to %")
    

    def set_speed(self, speed):
        if speed >= 0 and speed <= 100:
            self.direction(0)
            self.step.duty(int(1023/100*speed))
        else:
            print("Dalban, set speed to %")
    
