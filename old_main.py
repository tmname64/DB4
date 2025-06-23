# main.py #

from machine import Pin, I2C
from Controllers.IR_Sensor import LTR329
import utime
import time
import network

from Controllers.Cooler import Cooler
from Controllers.Pump import PumpMotor
from Controllers.Pump import Pump
from Network.WiFiManager import WiFiManager
from Network.AdafruitIOClient import AdafruitIOClient

from config import WIFI_PASSWORD, WIFI_SSID, ADAFRUIT_AIO_KEY, ADAFRUIT_AIO_USERNAME

from Controllers.LightSensor import LightSensor




def handle_cooling_toggle(val):
    if val in ("1", "ON"):
        print("Cooling -> ON")
    elif val in ("0", "OFF"):
        print("Cooling -> OFF")
    else:
        print("Unknown toggle value:", val)


def handle_pump_toggle(val):
    if val in ("1", "AIR"):
        print("Pump -> AIR")
    elif val in ("0", "FEED"):
        print("Pump -> FEED")
    else:
        print("Unknown toggle value:", val)







#########################################################
wifi = WiFiManager(WIFI_SSID, WIFI_PASSWORD)
if wifi.connect():
    print(wifi.ifconfig())
else:
    print("Running offline.")


aio = AdafruitIOClient(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY, wifi)
if aio.connect():
    print("Connected to Adafruit IO Dashboard")
    aio.publish('on-slash-off-cooling', 'ON')
    aio.publish('on-slash-off-toggle', 0)
    aio.publish('water-temperature', 12)

    aio.subscribe("on-slash-off-cooling", handle_cooling_toggle)
    aio.subscribe("on-slash-off-toggle", handle_pump_toggle)

else:
    print("!!! Failed to Connect to Adafruit IO")


print("Entering Main Loop")
while True:
    aio.check_msg()
    time.sleep(1)
#########################################################



# cooler = Cooler(4, 5)










#########################################################
# pump = PumpMotor(14, 27, 50)

# pump.moveBackward(2)
# time.sleep(4)
# pump.moveForward(4) 
#########################################################














#########################################################
# pump = Pump(15, 33, 1)

# pump.set_speed(1)
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
# i2c = I2C(0, scl=Pin(22), sda=Pin(23), freq=100000)
# sensor = LTR329(i2c)

# while True:
#     ch0, ch1 = sensor.read_lux()
#     print("CH0 (vis+IR):", ch0, "  CH1 (IR):", ch1)
#     time.sleep(1)

