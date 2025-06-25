import builtins, os, utime, time
from machine import I2C, Pin, PWM
import machine

# ───────── Local modules ─────────
from Controllers.OD_Sensor         import LTR329, LightMonitor
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


timeAndDate = TimeAndDate(
    year=2025, 
    month=6, 
    day=24, 
    dayOfWeek=2, 
    hour=15, 
    minutes=40
)

is_air = is_air_initially

FORCED_FEED = False



######## Logging ###########
DATA_DIR            = 'data'
CSV_LOG             = f'{DATA_DIR}/data_log.csv'
TEMP_PID_LOG        = f'{DATA_DIR}/temp_PID_log.csv'




def _init_log(path, header):
    with open(path, "w") as f:
        f.write(header + "\n")

_init_log(TEMP_PID_LOG,"Time,Actuator,Avg_Temperature,Kp,Ki,Kd,Cooler_Pump_Power")
_init_log(CSV_LOG, "Time,Concentration,Time_to_feed")



RUN = True
def STOP_program():
    global RUN ; RUN = False

 

INTERVAL_ACTIVATION = 60  
INTERVAL_FEEDING    = 30    # push cadence (in minutes)
WIFI_RETRY_SECONDS  = 10
AIO_RETRY_SECONDS   = 20



print('[SYS] Initialising hardware …')
pump                = Pump(pinDirection=5, pinStep=17, speed=0)
pumpCooler          = Pump(pinDirection=18, pinStep=19, speed=0)
pumpMotor           = PumpMotor(4, 16, 50)
i2c                 = machine.I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)
sensor              = LTR329(i2c)
odSensor            = LightMonitor(sensor, led_pin=26)
cooler              = Cooler(pinPower=33, pinFan=25)
temperatureSensor   = TemperatureSensor(32)
oled                = OLED(pinScl=15, pinSda=13)
print('[SYS] Hardware ready')




PID = PID(temperatureSensor.read_temp(), target_temperature) # Target temperature
PID.setProportional(8.5)
PID.setIntegral(3)
PID.setDerivative(0.5)




# ───────── Network setup ─────────
TERMINAL_FEED = 'terminal'

wifi = WiFiManager(WIFI_SSID, WIFI_PASSWORD)
print('[NET] Connecting to Wi-Fi ' + WIFI_SSID + ' …')
wifi.connect()
aio = AdafruitIOClient(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY, wifi)
if not aio.connect():
    print('[ERR] AIO connect failed')
else:
    print('[NET] AIO connected')


# ─── subscribe to remote pump commands ───
def handle_pump_feed(val):
    global FORCED_FEED
    if str(val).upper() == 'FEED':
        FORCED_FEED = True
    else:
        FORCED_FEED = False
    print(f"[NET] pump command -> {'FEED' if FORCED_FEED else 'AIR'}")


def handle_desired_temp(val):
    try:
        new_setpoint = int(val)
        PID.setDesiredTemperature(new_setpoint)
        print(f"[NET] desired-temperature -> {new_setpoint}")
    except Exception:
        print(f"[ERR] bad desired-temperature payload: {val}")



aio.subscribe('pump', handle_pump_feed)
aio.subscribe('desired-temperature', handle_desired_temp)





def read_temperature(temperatureSensor):
    temperatures = [] 
    i = 15 # Samples to average on (Note: read_temp() already averages over 50 readings)
    for _ in range(i):
        temperatures.append(temperatureSensor.read_temp())
    newTemp = sum(temperatures)/i
    return newTemp


def switch_pump(is_air, initial_pump_speed=100):
    pump.set_speed(0)
    time.sleep(1)

    if is_air:
        pumpMotor.moveForward(5) 
        print('[SYS] Switching to Air Pump', end=' ')
    else:
        pumpMotor.moveBackward(3)
        print('[SYS] Switching to Feed Pump', end=' ')

    time.sleep(5)

    print('[SYS] Pump switched ')
    pump.set_speed(initial_pump_speed)

    return not is_air

    

def adjustSpeedCoolerPump(outputPID):
    if outputPID <= 2:
        cooler.LowPower() ; cooler_pump_power = 1
        pumpCooler.set_speed(cooler_pump_power)
        
    elif outputPID <= 12:
        cooler.HighPower() ; cooler_pump_power = int(outputPID/12*100)
        pumpCooler.set_speed(cooler_pump_power)

    else:
        cooler_pump_power = 100
        pumpCooler.set_speed(cooler_pump_power)
    
    return cooler_pump_power



def compute_feeding_time(OD_measure):
    Ca = OD_measure
    Cm = 5000
    flow = 3
    Vm = 4000 # 4L

    dT = Cm * Vm / (Ca * flow)
    return dT


def feed_mussels(last_concentration, forced_feed):
    global is_air

    if is_air == False:
        is_air = switch_pump(False)
    
    conc = odSensor.measureConcentration(last_concentration, forced_feed)

    print("Algae Concentration: " + str(conc))

    ### ADD HERE CHECK OF CONCENTRATION

    dT = compute_feeding_time(conc)    # seconds

    print("Time to Feed: " + str(dT) + "s")

    timeFeedingMussels = utime.ticks_ms()

    print("[SYS] Starting feeding mussels …")

    while True:
        if utime.ticks_diff(utime.ticks_ms(), timeFeedingMussels) >= dT * 1000:
            print("[SYS] Finished feeding mussels !")
            is_air = switch_pump(True)
            break

    return conc, dT

    

def write_PID_data(actuator, avg_temp, pid_obj, pump_power):
    row = ",".join(map(str, [
        timeAndDate.date_time(),
        round(actuator, 2),
        round(avg_temp, 2),
        pid_obj.P,
        pid_obj.I,
        pid_obj.D,
        pump_power
    ]))
    with open(TEMP_PID_LOG, "a") as f:
        f.write(row + "\n")



def write_data(conc, dT):
    row = ",".join(map(str, [
        timeAndDate.date_time(),
        round(conc),
        round(dT, 2)
    ]))
    with open(CSV_LOG, "a") as f:
        f.write(row + "\n")




def stop_system():
    global is_air
    print("\n-------------------------------------------------")
    is_air = switch_pump(True)
    pump.set_speed(0)
    pumpCooler.set_speed(0)
    odSensor.turnOffLED()
    print("------------[SYS]--System stopped!!!!!!---------------")



initalTemperature = temperatureSensor.read_temp()
initalActuatorValue = PID.update(initalTemperature)
adjustSpeedCoolerPump(initalActuatorValue)




display_data_counter = 29
feedCounter = 0

conc = 295712
last_concentration = conc


aio.send_bulk({'desired-temperature': target_temperature})

pump_air = False

timerActivation = utime.ticks_ms()

print('[SYS] Entered Main Loop')
while RUN:
    newTemp = read_temperature(temperatureSensor)

    
    actuatorValue = PID.update(newTemp)
    
    if utime.ticks_diff(utime.ticks_ms(), timerActivation) >= INTERVAL_ACTIVATION * 1000:
        display_data_counter += 1
        cooler_pump_power = adjustSpeedCoolerPump(actuatorValue)
        timerActivation = utime.ticks_ms()


        if pump_air:
            pump.set_speed(94)
        else:
            pump.set_speed(0)
        
        pump_air = not pump_air

        print("\n\nTime: " + str(timeAndDate.date_time()))
        print("Actuator: " + str(actuatorValue))
        print("Avg Temperature: " + str(newTemp))
        print("PID Values: " + PID.overviewParameters)

        oled.display_PID_controls( 
            round(newTemp, 2),
            f"{int(conc/1000)}k",         
            timeAndDate.date_time(),
            is_air
        )
        
        write_PID_data(actuatorValue, newTemp, PID, cooler_pump_power)

        aio.send_bulk({'water-temperature': round(newTemp, 2)})


        if display_data_counter == 30 or FORCED_FEED:
            display_data_counter = 0
            feedCounter += 1
            last_concentration = conc

            aio.send_bulk({'pump': 'FEED'})
            conc, dT = feed_mussels(last_concentration, FORCED_FEED)
            
            FORCED_FEED = False
            
            write_data(conc, dT)

            lines = [
                f"--------- {timeAndDate.date_time()} ---------",
                f"Temperature: {newTemp:.2f} C",
                f"Concentration: {int(conc/1000)}k",
                f"Pumped Algae for {dT:.1f} sec"

            ]
            terminal_msg = "\n".join(lines)

            payload = {
                TERMINAL_FEED:     terminal_msg,
                'concentration':     int(conc),
                'pump': 'AIR'
            }

            if not aio.send_bulk(payload):
                print('[ERR] send_bulk failed, retrying…')
                time.sleep(AIO_RETRY_SECONDS)
                aio.connect()

            if feedCounter == 20:
                STOP_program()


stop_system() 