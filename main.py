import builtins, os, utime, time
from machine import I2C, Pin, PWM
import machine

# ───────── Local modules ─────────
from Controllers.IR_Sensor         import DualWavelengthOD, LTR329, LightMonitor
from Controllers.TemperatureSensor import TemperatureSensor
from Controllers.Cooler            import Cooler
from Controllers.Pump              import PumpMotor, Pump
from Controllers.PID_ar            import PID
from Controllers.TimeAndDate       import TimeAndDate
from Network.WiFiManager           import WiFiManager
from Network.AdafruitIOClient      import AdafruitIOClient
from config import (
    WIFI_SSID, WIFI_PASSWORD,
    ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY, 
    target_temperature, initial_cooler_mode,
    is_air_initially, mussel_nr
)


timeAndDate = TimeAndDate(
    year=2025, 
    month=6, 
    day=24, 
    dayOfWeek=2, 
    hour=11, 
    minutes=40
)



is_air = is_air_initially


######## MAIN ###########

RUN = True
def STOP_program():
    global RUN
    RUN = False



DATA_DIR            = 'data'
CSV_LOG             = f'{DATA_DIR}/data_log.csv'
TEMP_PID_LOG        = f'{DATA_DIR}/temp_PID_log.csv'
INTERVAL_ACTIVATION = 60  
INTERVAL_FEEDING    = 30    # push cadence (in minutes)
WIFI_RETRY_SECONDS  = 60
AIO_RETRY_SECONDS   = 60






print('[SYS] Initialising hardware …')
pump = Pump(pinDirection=5, pinStep=17, speed=0)
pumpCooler = Pump(pinDirection=18, pinStep=19, speed=0)
pumpMotor = PumpMotor(4, 16, 50)
i2c = machine.I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)
sensor = LTR329(i2c)
odSensor = LightMonitor(sensor, led_pin=26)
cooler = Cooler(pinPower=33, pinFan=25)
temperatureSensor = TemperatureSensor(32)
print('[SYS] Hardware ready')




PID = PID(temperatureSensor.read_temp(), target_temperature) # Terget temperature
PID.setProportional(8.5)
PID.setIntegral(3)
PID.setDerivative(0.5)




def read_temperature(temperatureSensor):
    temperatures = [] 
    i = 15 # Samples to average on (Note: read_temp() already averages over 50 readings)
    for _ in range(i):
        temperatures.append(temperatureSensor.read_temp())
    newTemp = sum(temperatures)/i - 1
    return newTemp


def switch_pump(is_air, initial_pump_speed=100):
    pump.set_speed(0)
    time.sleep(1)

    is_air = not is_air

    if is_air:
        pumpMotor.moveForward(5) 
        print('[SYS] Switching to Air Pump', end=' ')
    else:
        pumpMotor.moveBackward(5)
        print('[SYS] Switching to Feed Pump', end=' ')

    time.sleep(5)

    print('[SYS] Pump switched ')
    pump.set_speed(initial_pump_speed)
    return is_air
    

def adjustSpeedCoolerPump(outputPID):
    if outputPID <= 2:
        cooler.LowPower()
        pumpCooler.set_speed(1)

    elif outputPID <= 12:
        cooler.HighPower()
        pumpCooler.set_speed(int(outputPID/12*100))

    else:
        cooler.fanOn()
        cooler.HighPower()
        pumpCooler.set_speed(100)


def compute_feeding_time(OD_measure):
    Ca = OD_measure
    Cm = 4900
    flow = 3
    Vm = 4000 # 4L

    dT = Cm * Vm / (Ca * flow)
    return dT


def feed_mussels():
    if is_air == True:
        switch_pump(is_air)
    
    conc = odSensor.measureConcentration()

    print("Algae Concentration: " + str(conc))

    ### ADD HERE CHECK OF CONCENTRATION

    dT = compute_feeding_time(conc)    # seconds

    print("Time to Feed: " + str(dT) + "s")

    timeFeedingMussels = utime.ticks_ms()

    print("[SYS] Starting feeding mussels …")

    while True:
        if utime.ticks_diff(utime.ticks_ms(), timeFeedingMussels) >= dT * 1000:
            print("[SYS] Finished feeding mussels !")
            switch_pump(is_air)
            break

    



def write_PID_data():
    pass



def write_data():
    pass




def stop_system():
    global is_air
    print("\n-------------------------------------------------")
    is_air = switch_pump(True)
    cooler.fanOff()
    pump.set_speed(0)
    pumpCooler.set_speed(0)
    odSensor.turnOffLED()
    print("------------------System stopped!!!!!!---------------")



initalTemperature = temperatureSensor.read_temp()
initalActuatorValue = PID.update(initalTemperature)
adjustSpeedCoolerPump(initalActuatorValue)




display_data_counter = 29


timerActivation = utime.ticks_ms()

while RUN == True:
    newTemp = read_temperature(temperatureSensor)
    actuatorValue = PID.update(newTemp)
    
    if utime.ticks_diff(utime.ticks_ms(), timerActivation) >= INTERVAL_ACTIVATION * 1000:
        display_data_counter += 1
        adjustSpeedCoolerPump(actuatorValue)
        timerActivation = utime.ticks_ms()

        print("\n\nTime: " + str(timeAndDate.date_time()))
        print("Actuator: " + str(actuatorValue))
        print("Avg Temperature: " + str(newTemp))
        print("PID Values: " + PID.overviewParameters)

        if display_data_counter == 30:
            display_data_counter = 0
            feed_mussels()




stop_system()