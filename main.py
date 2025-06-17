# main.py #

from machine import Pin, I2C
from Controllers.IR_Sensor import LTR329
import utime
import time
import network

from Controllers.Cooler import Cooler
from Controllers.Pump import PumpMotor
from Controllers.LightSensor import LightSensor





#########################################################
# WIFI_SSID = "Mob"
# WIFI_PASSWORD = "Moldova1"

# def connect_wifi():
#     ap_if = network.WLAN(network.AP_IF)
#     ap_if.active(False)
#     wifi = network.WLAN(network.STA_IF)
#     wifi.active(True)
#     wifi.disconnect()
#     wifi.connect(WIFI_SSID, WIFI_PASSWORD)

#     print(WIFI_SSID)

#     if not wifi.isconnected():
#         print('Connecting..')
#         timeout = 0
#         while (not wifi.isconnected() and timeout < 20):
#             print(20 - timeout)
#             timeout = timeout + 1
#             time.sleep(1) # TODO: MAYBE NEED MORE TIME TO CONNECT?
#     if wifi.isconnected():
#         print('Connected')
#     else:
#         wifi.disconnect()
#         print("\nNot connected... Disconnected from the server, activities will run locally\n")

# connect_wifi()
#########################################################



# cooler = Cooler(4, 5)










#########################################################
# pump = PumpMotor(14, 27)


# # pump.set_direction(False)
# print("counter")
# for i in range(13):
#     print(i)
#     pump.oneStepForward()


# time.sleep(4)

# print("counter-counter")
# for i in range(13):
#     print(i)
#     pump.oneStepBackward()
#########################################################














#########################################################
# # cooler.fanOn()
# pump.set_speed(1)
# time.sleep(5)
# pump.set_speed(0)
# pump.switch_direction()
# time.sleep(5)
# pump.set_speed(1)


# cooler.fanOff()
# time.sleep(100)
#########################################################










# Try a slower bus speed if 400kHz fails:
i2c = I2C(0, scl=Pin(22), sda=Pin(23), freq=100000)
sensor = LTR329(i2c)

while True:
    ch0, ch1 = sensor.read_lux()
    print("CH0 (vis+IR):", ch0, "  CH1 (IR):", ch1)
    time.sleep(1)

