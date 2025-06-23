# main_network_dummy.py — streamlined dashboard‑test build
# -----------------------------------------------------------------------------
# 1. Publishes sensor data every **60 s** (INTERVAL_SECONDS = 60).
# 2. Listens to Adafruit IO feeds for cooler/pump/desired‑temperature changes and
#    prints a one‑liner when they arrive.
# 3. Dashboard payload only includes state fields when they actually change.
# 4. Terminal output still filtered to a few INFO tags.

import builtins, os, utime, time, random
from machine import Pin  # import parity; unused in dummy mode

# ───────── Configuration ─────────
DATA_DIR          = 'data'
CSV_LOG           = f'{DATA_DIR}/data_log.csv'
TERM_LOG          = f'{DATA_DIR}/terminal.log'
INTERVAL_SECONDS  = 60         # ← once per minute now
AIO_RETRY_SECONDS = 30
OD_TO_CONC_FACTOR = 0.85

# Initial dummy state (would come from config in real build)
current_pump_mode = 'AIR'
cooler_mode       = 'OFF'
desired_temp      = 25.0

# ───────── Minimal safe_print ─────────
_original_print = builtins.print
INFO_TAGS = ('[SYS]', '[LOOP]', '[ERR]', '[NET]')
_print_lock = False
TERMINAL_FEED = 'terminal'
aio = None  # set later

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

# ───────── Dummy sensors & actuators (silent) ─────────
class DummyCooler:  # fan / power no‑ops
    def fanOn(self): pass
    def fanOff(self): pass
    def HighPower(self): pass
    def LowPower(self): pass
    def is_on(self): return cooler_mode == 'ON'

class DummyPumpMotor:
    def moveBackward(self, s): pass
    def moveForward(self, s): pass

class DummyPump:
    def set_speed(self, s): pass

class DummyOD:
    blank_reference = 10_000
    def set_blank(self, n=3): pass
    def compute_od(self): return random.uniform(0.0, 1.5)

class DummyTempSensor:
    def read_temp(self): return random.uniform(20.0, 30.0)

Cooler = DummyCooler
PumpMotor = DummyPumpMotor
Pump = DummyPump
DualWavelengthOD = DummyOD
TemperatureSensor = DummyTempSensor

# ───────── Simplified networking stubs ─────────
from Network.WiFiManager      import WiFiManager
from Network.AdafruitIOClient import AdafruitIOClient

class ResilientAIO:
    def __init__(self, user, key, wifi):
        self.user = user; self.key = key; self.wifi = wifi
        self.client = None; self.ok = False; self.last_try = 0; self.subs = []

    def _connect(self):
        if time.time() - self.last_try < AIO_RETRY_SECONDS: return
        self.last_try = time.time()
        if not self.wifi.is_connected(): return
        try:
            self.client = AdafruitIOClient(self.user, self.key, self.wifi)
            self.ok = self.client.connect()
            if self.ok:
                for feed, cb in self.subs:
                    self.client.subscribe(feed, cb)
                print('[NET] AIO connected')
        except Exception:
            self.ok = False; print('[ERR] AIO connect failed')

    def is_connected(self):
        if not self.ok: self._connect()
        return self.ok

    def send_bulk(self, data: dict):
        if not self.is_connected() or not data: return False
        try:
            self.client.send_bulk(data)
            return True
        except Exception:
            self.ok = False; print('[ERR] send_bulk failed'); return False

    def subscribe(self, feed, cb):
        self.subs.append((feed, cb))
        if self.is_connected():
            try: self.client.subscribe(feed, cb)
            except Exception: pass

    def check_msg(self):
        if self.is_connected():
            try: self.client.check_msg()
            except Exception: self.ok = False

# ───────── FS helpers ─────────

def ensure_fs():
    try: os.mkdir(DATA_DIR)
    except OSError: pass
    if 'data_log.csv' not in os.listdir(DATA_DIR):
        with open(CSV_LOG, 'w') as f:
            f.write('timestamp,temperature,od,concentration\n')
    if 'terminal.log' not in os.listdir(DATA_DIR):
        open(TERM_LOG, 'w').close()

# ───────── Init dummy hardware ─────────

def init_hardware():
    return Cooler(), PumpMotor(), Pump(), Pump(), DualWavelengthOD(), TemperatureSensor()

# ───────── CSV logger ─────────

def log_row(t, od, conc):
    ts = '{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}'.format(*utime.localtime()[:6])
    with open(CSV_LOG, 'a') as f:
        f.write(f"{ts},{t:.2f},{od:.2f},{conc:.2f}\n")

# ───────── Dashboard feed handlers ─────────

def handle_desired_temp(val):
    global desired_temp
    try:
        desired_temp = float(val)
        print(f'[NET] desired‑temperature -> {desired_temp:.1f}')
    except Exception:
        print('[ERR] bad desired‑temp payload')

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
        print(f'[NET] pump -> {mode}')

# ───────── Main loop ─────────

def main():
    global aio
    ensure_fs()
    cooler, pump_motor, pump, pumpCooler, odm, temp_sensor = init_hardware()

    wifi = WiFiManager('ssid', 'pass')  # replace creds if needed
    wifi.connect()
    aio = ResilientAIO('user', 'key', wifi)

    # subscribe to dashboard feeds
    aio.subscribe('desired-temperature', handle_desired_temp)
    aio.subscribe('cooler',              handle_cooler)
    aio.subscribe('pump',                handle_pump)
    aio.subscribe(TERMINAL_FEED,         lambda v: None)

    print('[SYS] Dummy loop running (60 s interval)')

    last_pub = time.ticks_ms()
    prev_state = {
        'cooler': cooler_mode,
        'pump': current_pump_mode,
        'desired‑temperature': desired_temp,
    }

    while True:
        aio.check_msg()

        if time.ticks_diff(time.ticks_ms(), last_pub) >= INTERVAL_SECONDS * 1000:
            last_pub = time.ticks_ms()
            t  = temp_sensor.read_temp()
            od = odm.compute_od()
            conc = od * OD_TO_CONC_FACTOR

            payload = {
                'water-temperature': t,
                'od-sensor': od,
                'concentration': conc,
            }
            for key, prev in prev_state.items():
                cur = {'cooler': cooler_mode,
                       'pump': current_pump_mode,
                       'desired‑temperature': desired_temp}[key]
                if cur != prev:
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
