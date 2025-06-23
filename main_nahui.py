# main_network.py — Hardware build (reduced logging, 60 s cadence)
# ==================================================================================
# * Publishes one telemetry batch per minute to Adafruit IO.
# * Only concise log lines with tags in INFO_TAGS are forwarded to terminal feed.
# * State keys (cooler/pump/desired‑temperature) are included in MQTT payload only
#   when they change, saving bandwidth.
# * Dashboard feed handlers update internal state and emit a brief `[NET] …` line.
# * PID control is active; PID output logged once per cycle when cooler is ON.
# * File‑system logging (CSV + terminal.log) retained.

import builtins, os, utime, time
from machine import I2C, Pin

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

# ───────── Constants ─────────
DATA_DIR            = 'data'
CSV_LOG             = f'{DATA_DIR}/data_log.csv'
TERM_LOG            = f'{DATA_DIR}/terminal.log'
INTERVAL_SECONDS    = 60      # push cadence
WIFI_RETRY_SECONDS  = 15
AIO_RETRY_SECONDS   = 30
USE_PID             = False

OD_TO_CONC_FACTOR   = 0.85
PID_KP, PID_KI, PID_KD = 8.5, 3, 0.5

# ───────── Runtime state ─────────
desired_temp      = initial_desired_temperature
current_pump_mode = initial_pump_mode
cooler_mode       = initial_cooler_mode

wifi = aio = None  # will be set later
cooler = pump_motor = pump = pumpCooler = odm = temp_sensor = None  # handles

# ───────── selective safe_print ─────────
_original_print = builtins.print
INFO_TAGS = ('[SYS]', '[LOOP]', '[NET]', '[PID]', '[ERR]', '[FS]')
_print_lock = False
TERMINAL_FEED = 'terminal'

def safe_print(*args, **kwargs):
    if not args:
        return _original_print(*args, **kwargs)
    first = args[0]
    if not (isinstance(first, str) and first.startswith(INFO_TAGS)):
        return  # ignore noisy prints
    global _print_lock
    if _print_lock:
        return _original_print(*args, **kwargs)
    _print_lock = True
    try:
        _original_print(*args, **kwargs)
        ts = '{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}'.format(*utime.localtime()[:6])
        entry = ts + ' ' + ' '.join(str(a) for a in args)
        try:
            if aio is not None:
                aio.send_bulk({TERMINAL_FEED: entry})
        except Exception:
            pass
        try:
            with open(TERM_LOG, 'a') as f:
                f.write(entry + '\n')
        except Exception:
            pass
    finally:
        _print_lock = False

builtins.print = safe_print

# ───────── filesystem bootstrap ─────────

def _ensure_fs():
    try:
        os.mkdir(DATA_DIR)
    except OSError:
        pass
    if 'data_log.csv' not in os.listdir(DATA_DIR):
        with open(CSV_LOG, 'w') as f:
            f.write('timestamp,temperature,od,concentration\n')
    if 'terminal.log' not in os.listdir(DATA_DIR):
        open(TERM_LOG, 'w').close()
    print('[FS] filesystem ready')

# ───────── resilient Adafruit IO ─────────
class ResilientAIO:
    def __init__(self, user, key, wifi):
        self.user, self.key, self.wifi = user, key, wifi
        self.cli = None; self.ok = False; self.last_try = 0; self.subs = []
        print('[SYS] AIO wrapper initialised')

    def _connect(self):
        if time.time() - self.last_try < AIO_RETRY_SECONDS:
            return
        self.last_try = time.time()
        if not self.wifi.is_connected():
            return
        try:
            self.cli = AdafruitIOClient(self.user, self.key, self.wifi)
            self.ok = self.cli.connect()
            if self.ok:
                for feed, cb in self.subs:
                    self.cli.subscribe(feed, cb)
                print('[NET] AIO connected')
        except Exception:
            self.ok = False; print('[ERR] AIO connect failed')

    def is_connected(self):
        if not self.ok:
            self._connect()
        return self.ok

    def send_bulk(self, data: dict):
        if not self.is_connected() or not data:
            return False
        try:
            self.cli.send_bulk(data); return True
        except Exception:
            self.ok = False; print('[ERR] send_bulk failed'); return False

    def subscribe(self, feed, cb):
        self.subs.append((feed, cb))
        if self.is_connected():
            try:
                self.cli.subscribe(feed, cb)
            except Exception:
                pass

    def check_msg(self):
        if self.is_connected():
            try:
                self.cli.check_msg()
            except Exception:
                self.ok = False

# ───────── hardware init ─────────

def init_hardware():
    print('[SYS] Initialising hardware …')
    cooler = Cooler(pinPower=33, pinFan=25)
    pump_motor = PumpMotor(pinDirection=4, pinStep=16, interval=50)
    pump = Pump(pinDirection=5, pinStep=17, speed=0)
    pumpCooler = Pump(pinDirection=18, pinStep=19, speed=0)

    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100_000)
    ltr = LTR329(i2c)
    odm = DualWavelengthOD(ltr, vis_pin=19, ir_pin=18)
    temp_sensor = TemperatureSensor(32)
    print('[SYS] Hardware ready')
    return cooler, pump_motor, pump, pumpCooler, odm, temp_sensor

# ───────── CSV logging ─────────

def log_row(t, od, conc):
    ts = '{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}'.format(*utime.localtime()[:6])
    with open(CSV_LOG, 'a') as f:
        f.write(f"{ts},{t:.2f},{od:.2f},{conc:.2f}\n")

# ───────── Dashboard handlers ─────────

def handle_desired_temp(val):
    global desired_temp
    try:
        desired_temp = float(val)
        if USE_PID and pid:
            pid.setDesiredTemperature(desired_temp)
        print(f'[NET] desired-temperature -> {desired_temp:.1f}')
    except Exception:
        print('[ERR] bad desired-temp payload')


def handle_cooler(val):
    global cooler_mode
    mode = 'ON' if str(val).upper() in ('ON', '1') else 'OFF'
    if mode != cooler_mode:
        cooler_mode = mode
        print(f'[NET] cooler -> {mode}')


def handle_pump(val):
    global current_pump_mode
    mode = 'AIR' if str(val).upper() == 'AIR' else 'FEED'
    if mode != current_pump_mode:
        current_pump_mode = mode
        if mode == 'FEED':
            pump_motor.oneStepForward(4)
        elif mode == 'AIR':
            pump_motor.oneStepBackward(2)
        print(f'[NET] pump -> {mode}')

# ───────── PID setup ─────────
pid = None
if USE_PID:
    pid = PID(0, desired_temp)
    pid.setProportional(PID_KP)
    pid.setIntegral(PID_KI)
    pid.setDerivative(PID_KD)

def adjust_actuators(temp):
    if not (USE_PID and pid and cooler_mode == 'ON'):
        return
    out = pid.update(temp)
    print(f'[PID] out={out:.2f}')
    if out <= 2:
        cooler.LowPower(); pumpCooler.set_speed(0)
    else:
        cooler.HighPower(); pumpCooler.set_speed(100)

# ───────── Main loop ─────────

def main():
    global wifi, aio, cooler, pump_motor, pump, pumpCooler, odm, temp_sensor

    _ensure_fs()
    cooler, pump_motor, pump, pumpCooler, odm, temp_sensor = init_hardware()

    print('[SYS] Setting blank reference …')
    odm.set_blank(); print('[SYS] Blank reference set')

    wifi = WiFiManager(WIFI_SSID, WIFI_PASSWORD)
    wifi.connect()
    aio = ResilientAIO(ADAFRUIT_AIO_USERNAME, ADAFRUIT_AIO_KEY, wifi)

    # subscribe to dashboard commands
    aio.subscribe('desired-temperature', handle_desired_temp)
    aio.subscribe('cooler',              handle_cooler)
    aio.subscribe('pump',                handle_pump)
    aio.subscribe(TERMINAL_FEED,         lambda _v: None)

    # initial dashboard snapshot
    init_temp = temp_sensor.read_temp()
    aio.send_bulk({
        'water-temperature':   init_temp,
        'od-sensor':           0.0,
        'concentration':       0.0,
        'cooler':              cooler_mode,
        'pump':                current_pump_mode,
        'desired-temperature': desired_temp,
    })

    print('[SYS] Entering main loop (60‑s cadence)')

    last_pub = time.ticks_ms()
    prev_state = {
        'cooler': cooler_mode,
        'pump': current_pump_mode,
        'desired-temperature': desired_temp,
    }

    while True:
        aio.check_msg()

        if time.ticks_diff(time.ticks_ms(), last_pub) >= INTERVAL_SECONDS * 1000:
            last_pub = time.ticks_ms()
            t  = temp_sensor.read_temp()
            od = odm.compute_od()
            conc = od * OD_TO_CONC_FACTOR

            adjust_actuators(t)

            payload = {
                'water-temperature': t,
                'od-sensor':         od,
                'concentration':     conc,
            }
            # include state keys only if changed
            for key in ('cooler', 'pump', 'desired-temperature'):
                cur = {'cooler': cooler_mode,
                       'pump': current_pump_mode,
                       'desired-temperature': desired_temp}[key]
                if cur != prev_state[key]:
                    payload[key] = cur
                    prev_state[key] = cur

            aio.send_bulk(payload)
            log_row(t, od, conc)
            print(f'[LOOP] T={t:.1f}°C OD={od:.2f} C={conc:.2f}')

        time.sleep_ms(50)

# ───────── Entry guard ─────────
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('[SYS] KeyboardInterrupt — exit')
