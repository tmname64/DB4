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
from Controllers.Display            import OLED 
from Network.WiFiManager           import WiFiManager
from Network.AdafruitIOClient      import AdafruitIOClient
from config import (
    WIFI_SSID, WIFI_PASSWORD,
    ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY, 
    target_temperature, initial_cooler_mode,
    is_air_initially, mussel_nr
)

if not os.path.exists('data'):
    os.mkdir('data')


timeAndDate = TimeAndDate(
    year=2025, 
    month=6, 
    day=24, 
    dayOfWeek=2, 
    hour=11, 
    minutes=40
)

is_air = is_air_initially

######## Logging ###########
DATA_DIR            = 'data'
CSV_LOG             = f'{DATA_DIR}/data_log.csv'
TEMP_PID_LOG        = f'{DATA_DIR}/temp_PID_log.csv'

os.makedirs(DATA_DIR, exist_ok=True)

def _init_log(path, header):
    with open(path, "w") as f:
        f.write(header + "\n")

_init_log(CSV_LOG,"Time,Actuator,Avg_Temperature,Kp,Ki,Kd,Cooler_Pump_Power")
_init_log(TEMP_PID_LOG, "Time,Temperature,PID_Values")

######## Screen ###########

oled = OLED(pinScl=22, pinSda=21) # from the screen file


def pretty_datetime(): #from chat since i cant test the formatting
    return "{:02d}-{:02d} {:02d}:{:02d}".format(
        timeAndDate.day, timeAndDate.month,
        timeAndDate.hour, timeAndDate.minutes
    )



######## MAIN ###########

RUN = True
def STOP_program():
    global RUN
    RUN = False




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

    if is_air:
        pumpMotor.moveForward(5) 
        print('[SYS] Switching to Air Pump', end=' ')
    else:
        pumpMotor.moveBackward(5)
        print('[SYS] Switching to Feed Pump', end=' ')

    time.sleep(5)

    print('[SYS] Pump switched ')
    pump.set_speed(initial_pump_speed)
    return not is_air ##bug here i fixed
    

def adjustSpeedCoolerPump(outputPID):
    if outputPID <= 2:
        cooler.LowPower()
        cooler_pump_power = 1
        pumpCooler.set_speed(cooler_pump_power)


    elif outputPID <= 12:
        cooler.HighPower()
        cooler_pump_power = int(outputPID/12*100)
        pumpCooler.set_speed(cooler_pump_power)

    else:
        cooler.fanOn()
        cooler.HighPower()
        cooler_pump_power = 100
        pumpCooler.set_speed(cooler_pump_power)
    
    return cooler_pump_power


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

    return conc, dT

    

def time_right_now():
    return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        timeAndDate.year, timeAndDate.month, timeAndDate.day,
        timeAndDate.hour, timeAndDate.minutes, int(time.time() % 60))


def write_PID_data(actuator, avg_temp, pid_obj, pump_power):
    row = ",".join(map(str, [
        time_right_now(),
        round(actuator, 2),
        round(avg_temp, 2),
        pid_obj.Kp,
        pid_obj.Ki,
        pid_obj.Kd,
        pump_power
    ]))
    with open(CSV_LOG, "a") as f:
        f.write(row + "\n")



def write_data(conc, dT):
    row = ",".join(map(str, [
        time_right_now(),
        round(conc, 1),
        int(dT)
    ]))
    with open(TEMP_PID_LOG, "a") as f:
        f.write(row + "\n")




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
        cooler_pump_power = adjustSpeedCoolerPump(actuatorValue)
        timerActivation = utime.ticks_ms()

        print("\n\nTime: " + str(timeAndDate.date_time()))
        print("Actuator: " + str(actuatorValue))
        print("Avg Temperature: " + str(newTemp))
        print("PID Values: " + PID.overviewParameters)

        oled.display_PID_controls( #From display file
            temperature = round(newTemp, 1),
            concentration = round(conc if display_data_counter == 0 else 0, 1),
            frequency = INTERVAL_FEEDING,          
            dateAndTime = pretty_datetime()
        )
        
        write_PID_data(actuatorValue, newTemp, PID, cooler_pump_power)

        if display_data_counter == 30:
            display_data_counter = 0
            conc, dT = feed_mussels()
            write_data(conc, dT)

stop_system()