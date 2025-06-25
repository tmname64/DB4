import builtins, os, utime, time
from machine import I2C, Pin, PWM
from math import log10, isclose, log, exp


# ───────── Local modules ─────────
# from Controllers.IR_Sensor         import LTR329
from Controllers.TemperatureSensor import TemperatureSensor
from Controllers.Cooler            import Cooler
from Controllers.Pump              import PumpMotor, Pump
from Controllers.PID_ar            import PID
from Network.WiFiManager           import WiFiManager
from Network.AdafruitIOClient      import AdafruitIOClient
from Controllers.Display import OLED
from config import (
    WIFI_SSID, WIFI_PASSWORD,
    ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY, initial_cooler_mode
)



from config import conc_cal, od_cal



class LTR329:
    ADDRESS      = 0x29
    PARTID_REG   = 0x86
    MANUID_REG   = 0x87
    CONTROL_REG  = 0x80
    STATUS_REG   = 0x8C
    MEAS_RATE_REG= 0x85
    CH1_L        = 0x88
    CH1_H        = 0x89
    CH0_L        = 0x8A
    CH0_H        = 0x8B

    def __init__(self, i2c: I2C):
        self.i2c = i2c

        pid = self._read8(self.PARTID_REG)
        mid = self._read8(self.MANUID_REG)
        if pid != 0xA0 or mid != 0x05:
            raise RuntimeError(f"Found PARTID=0x{pid:02X}, MANUID=0x{mid:02X}; not an LTR329")

        self._write8(self.CONTROL_REG, 0x02)    # set RESET bit
        time.sleep(0.01)

        self._write8(self.CONTROL_REG, 0x01)
        time.sleep(0.1)
        

    def _write8(self, reg, val):
        self.i2c.writeto_mem(self.ADDRESS, reg, bytes([val]))

    def _read8(self, reg):
        return int.from_bytes(self.i2c.readfrom_mem(self.ADDRESS, reg, 1), "little")

    def read_lux(self):
        st = self._read8(self.STATUS_REG)
        if st & 0x80:  # bit7 = data invalid
            return (0, 0)

        lo1 = self._read8(self.CH1_L)
        hi1 = self._read8(self.CH1_H)
        lo0 = self._read8(self.CH0_L)
        hi0 = self._read8(self.CH0_H)

        ch1 = (hi1 << 8) | lo1
        ch0 = (hi0 << 8) | lo0
        return (ch0, ch1)


class LightMonitor:
    def __init__(self, sensor, led_pin):
        self.sensor = sensor
        self.led = Pin(led_pin, Pin.OUT)
        self.reference = 135   ### CHANGE LATER
        self.ODerror = 3       ### CHANGE LATER
        
        self.led.value(1)

    def read_intensity(self):
        ch0, ch1 = self.sensor.read_lux()
        return ch0
    
    def turnOffLED(self):
        self.led.value(0)

    def turnOnLED(self):
        self.led.value(0)

    def measureOD(self):
        OD_vals = []

        wrongMeasureCounter = 0

        oldOD = -1
        n = 15 # Samples to average on (Note: read_temp() already averages over 50 readings)
        for i in range(n):
            intensity = self.read_intensity()
            rawOD = (log10(intensity / self.reference)) * (-1)
            if oldOD == -1 or (abs(rawOD - oldOD) < self.ODerror and rawOD != 0):
                OD_vals.append(rawOD)
                oldOD = rawOD
            if abs(rawOD - oldOD) >= self.ODerror:
                wrongMeasureCounter += 1
                i -= 1

            if wrongMeasureCounter == 7: ### Wrong measurement check !!!
                return -1
            
            time.sleep(0.1)


        # print(OD_vals)    
        OD_value = sum(OD_vals)/n
    
        return OD_value
    
    def measureAccurateOD(self):
        OD_value = -1
        while OD_value == -1:
            OD_value = self.measureOD()
        
        return OD_value
    
    def measureConcentration(self):
        # ---  Fit log(conc) = ln(A) + B*OD  ---------------------------------------
        # compute means
        n = len(conc_cal)
        mean_od     = sum(od_cal) / n
        mean_lnconc = sum(log(c) for c in conc_cal) / n

        # least-squares slope B and intercept ln(A)
        num = sum((od_cal[i] - mean_od) * (log(conc_cal[i]) - mean_lnconc) 
                  for i in range(n)
              )
        den = sum((od_cal[i] - mean_od)**2 for i in range(n))
        B   = num / den
        lnA = mean_lnconc - B * mean_od
        A   = exp(lnA)

        
        # --- Computing concentration ----------------------
        return A * exp(B * self.measureAccurateOD)

    





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
# cooler = Cooler(pinPower=33, pinFan=25)


# print("Cool")
# cooler.fanOn()

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



# pump.set_speed(0)




# import machine
# import time


# i2c = machine.I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)
# sensor = LTR329(i2c)
# odSensor = LightMonitor(sensor, led_pin=26)


# pump = Pump(pinDirection=5, pinStep=17, speed=100)
# time.sleep(4)


# pump.set_speed(95)


# i=0
# while i<1500:
#     odValue = odSensor.measureAccurateOD()
#     print("Raw OD:", odValue)


#     intensity = odSensor.read_intensity()
#     print("Intensity:", intensity)

#     print()

#     time.sleep(1)
#     i += 1

# pump.set_speed(0)


# pumpCooler = Pump(pinDirection=18, pinStep=19, speed=100)

# time.sleep(30) 

# pumpCooler.set_speed(0)

# print('pump stopped')

#####################################################################






print('[SYS] Hardware ready')




# with open('data/PID_data.csv', 'w') as f:
#     f.write('timestamp_ms,actuator,avg_temp,Kp,Ki,Kd,pump_power\n')


# while RUN:
#     if utime.ticks_diff(utime.ticks_ms(), timerActivation) >= 10 * 1000:
#         timerActivation = utime.ticks_ms()

#         now = utime.ticks_ms()

import machine
    
# pump = Pump(pinDirection=5, pinStep=17, speed=0)
# pumpMotor = PumpMotor(4, 16, 50)
# i2c = machine.I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)
# sensor = LTR329(i2c)
# odSensor = LightMonitor(sensor, led_pin=26)

# is_air = True
         
# def switch_pump(is_air, initial_pump_speed=100):
#     pump.set_speed(0)
#     time.sleep(1)

#     if is_air:
#         pumpMotor.moveForward(5) 
#         print('[SYS] Switching to Air Pump', end=' ')
#     else:
#         pumpMotor.moveBackward(3)
#         print('[SYS] Switching to Feed Pump', end=' ')

#     time.sleep(5)

#     print('[SYS] Pump switched ')
#     pump.set_speed(initial_pump_speed)
#     return not is_air ##bug here i fixed
    


 
# pump.set_speed(100)

# time.sleep(6)

# is_air = switch_pump(is_air, 100)

# time.sleep(8)

# is_air = switch_pump(is_air)

# time.sleep(1)

# pump.set_speed(0) 


oled = OLED(pinScl=15, pinSda=13)


oled.display_PID_controls( #From display file
            temperature = round(17.13, 2),
            concentration = "293k",         
            dateAndTime = "24 Jun - 20:39",
            is_air=True
        )


