import builtins, os, utime, time
from machine import I2C, Pin, PWM

# ───────── Local modules ─────────
from Controllers.IR_Sensor         import DualWavelengthOD, LTR329
from Controllers.TemperatureSensor import TemperatureSensor
from Controllers.Cooler            import Cooler
# from Controllers.Pump              import PumpMotor, Pump
from Controllers.PID_ar            import PID
from Network.WiFiManager           import WiFiManager
from Network.AdafruitIOClient      import AdafruitIOClient
from config import (
    WIFI_SSID, WIFI_PASSWORD,
    ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY,
    initial_pump_mode, initial_desired_temperature, initial_cooler_mode
)




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
            # self.oneStepBackward()
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




#####################################################################
# pumpCooler = Pump(pinDirection=18, pinStep=19, speed=100)

# print("Pump Started")
#####################################################################


    






#####################################################################
# pump = Pump(pinDirection=5, pinStep=17, speed=100)

# time.sleep(5)
# print("50%")
# pump.set_speed(90)


# time.sleep(5)
# print("25%")
# pump.set_speed(55)
#####################################################################









#####################################################################
print('Helou')
pumpMotor = PumpMotor(4, 16, 50)
pumpMotor.moveForward(4)
#####################################################################
