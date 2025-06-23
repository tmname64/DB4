import builtins, os, utime, time
from machine import I2C, Pin, PWM

# ───────── Local modules ─────────
from Controllers.IR_Sensor         import DualWavelengthOD, LTR329
from Controllers.TemperatureSensor import TemperatureSensor
from Controllers.Cooler            import Cooler
from Controllers.Pump              import PumpMotor, Pump
from Controllers.PID_ar            import PID
from Network.WiFiManager           import WiFiManager
from Network.AdafruitIOClient      import AdafruitIOClient
from config import (
    WIFI_SSID, WIFI_PASSWORD,
    ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY,
    initial_pump_mode, initial_desired_temperature, initial_cooler_mode
)





#####################################################################
# pumpCooler = Pump(pinDirection=18, pinStep=19, speed=100)

# print("Pump Started")
#####################################################################


    






################ --- MOSHNII PUMP --- ##################################
# pump = Pump(pinDirection=5, pinStep=17, speed=100)

# time.sleep(5)
# print("50%")
# pump.set_speed(90)


# time.sleep(5)
# print("25%")
# pump.set_speed(55)
#####################################################################









#####################################################################
# print('Helou')
# pumpMotor = PumpMotor(4, 16, 50)


# pumpMotor.moveForward(5) 

# time.sleep(3)

# pumpMotor.moveBackward(5)


#####################################################################

 




################ --- MOSHNII PUMP + motor --- ##################################
# pump = Pump(pinDirection=5, pinStep=17, speed=100)
# pumpMotor = PumpMotor(4, 16, 50)

# pump.set_speed(85)
# print('Pump - Feed')


# time.sleep(5)

# print('Switching Pump')
# pump.set_speed(0)
# time.sleep(1)
# pumpMotor.moveForward(5) 
# time.sleep(3)


# print('Pump - AIR')
# pump.set_speed(85)

# print('Pump - STOP')
# time.sleep(6)
# pump.set_speed(0)
  
#####################################################################








# temp_sensor = TemperatureSensor(32)


# while True:
#     print(temp_sensor.read_temp())
#     time.sleep(1)










################ --- Measure Pump Flow --- ##################################

# pump = Pump(pinDirection=5, pinStep=17, speed=100)
# time.sleep(10)
# pump.set_speed(0)

#####################################################################








################ --- Cooler --- ##################################
cooler = Cooler(pinPower=33, pinFan=25)


print("Cool")
cooler.fanOn()

# time.sleep(5)

# cooler.fanOn()
#####################################################################

 







################ --- LED --- ##################################
# i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)

# ltr = LTR329(i2c)

# odm = DualWavelengthOD(ltr, vis_pin=34, ir_pin=18)


# print('LED nahui')

# vis_led = Pin(26, Pin.OUT)
# vir_led = Pin(27, Pin.OUT)


# vir_led.value(0)
# vis_led.value(1)
# time.sleep(5)


# vir_led.value(1)
# vis_led.value(0)


# time.sleep(5)

# vir_led.value(0)
# vis_led.value(0)

#####################################################################







################ --- OD Sensor --- ##################################

pump = Pump(pinDirection=5, pinStep=17, speed=100)


print('pump started')
pump.set_speed(100)
time.sleep(2)
pump.set_speed(85)

time.sleep(2)

# pump.set_speed(0)



class LightMonitor:
    def __init__(self, sensor, led_pin):
        self.sensor = sensor
        self.led = Pin(led_pin, Pin.OUT)
        # turn LED on and leave it on
        self.led.value(1)

    def read_intensity(self):
        self.led.value(1)
        ch0, ch1 = self.sensor.read_lux()
        return ch0


import machine, time
i2c = machine.I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)
sensor = LTR329(i2c)

monitor = LightMonitor(sensor, led_pin=26)
# odm = DualWavelengthOD(sensor, uv_pin=26, scatter_pin=27, pulse_ms=200)

# V0 = odm.set_blank()
# print("Blank reference:" + str(V0))


# V0 = odm.set_blank(samples=10)
# print("Blank reference V₀ =", V0)



i=0
while i<250:
    intensity = monitor.read_intensity()
    print("Raw intensity (ch0):", intensity)
    time.sleep(1)
    i += 1

pump.set_speed(0)



# pumpCooler = Pump(pinDirection=18, pinStep=19, speed=100)

# time.sleep(30) 

# pumpCooler.set_speed(0)

# print('pump stopped')

#####################################################################
