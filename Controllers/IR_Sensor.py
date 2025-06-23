from math import log10, isclose, log, exp
from machine import I2C, Pin
from time import sleep
import time


from config import conc_cal, od_cal


class LightMonitor:
    def __init__(self, sensor, led_pin):
        self.sensor = sensor
        self.led = Pin(led_pin, Pin.OUT)
        self.reference = 130   ### CHANGE LATER
        self.ODerror = 5       ### CHANGE LATER
        
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
            intensity = self.readIntensity()
            rawOD = (log10(intensity / self.reference))
            if oldOD == -1 or (abs(rawOD - oldOD) < self.ODerror and rawOD != 0):
                OD_vals.append(rawOD)
                oldOD = rawOD
            if abs(rawOD - oldOD) >= self.ODerror:
                wrongMeasureCounter += 1
                i -= 1

            if wrongMeasureCounter == 7: ### Wrong measurement check !!!
                return -1
            
            time.sleep(0.1)
            
        OD_value = sum(OD_vals)/n - 1
    
        return OD_value
    
    def measureAccurateOD(self):
        OD_value = -1
        while OD_value == -1:
            OD_value = self.measure_OD()
        
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

    


class DualWavelengthOD:
    def __init__(self, sensor, uv_pin, scatter_pin, blank_reference=None, pulse_ms=200):
        self.sensor = sensor
        self.uv_led = Pin(uv_pin, Pin.OUT)
        self.scatter_led = Pin(scatter_pin, Pin.OUT)
        self.blank_reference = blank_reference
        self.pulse_ms = pulse_ms
        # LEDs off
        self.uv_led.value(0)
        self.scatter_led.value(0)

    def _pulse_and_read(self, led: Pin) -> int:
        led.value(1)
        time.sleep_ms(self.pulse_ms)
        ch0, _ = self.sensor.read_lux()      # use channel-0 for UV & scatter
        led.value(0)
        return ch0

    def measure_intensities(self) -> tuple[int, int]:
        T_uv = self._pulse_and_read(self.uv_led)
        time.sleep_ms(50)  # small gap
        S_vis = self._pulse_and_read(self.scatter_led)
        return T_uv, S_vis

    def set_blank(self, samples:int=5) -> float:
        vals = []
        for _ in range(samples):
            T_uv, S_vis = self.measure_intensities()
            vals.append(T_uv - S_vis)
            time.sleep_ms(100)
        self.blank_reference = sum(vals) / len(vals)
        return self.blank_reference

    def _safe_od(self, V: float) -> float:
        # clamp invalid/negative to 0.0
        if V <= 0 or not self.blank_reference or self.blank_reference <= 0:
            return 0.0
        od = -log10(V / self.blank_reference)
        # fix “-0.0” → “0.0”
        return 0.0 if isclose(od, 0.0, abs_tol=1e-6) else od

    def read_all(self) -> dict:
        T_uv, S_vis = self.measure_intensities()
        Vcorr = T_uv - S_vis
        return {
            'T_uv':        T_uv,
            'S_scatter':   S_vis,
            'V_corrected': Vcorr,
            'OD':          self._safe_od(Vcorr)
        }


### Example usage ###
# import machine, time
# i2c = machine.I2C(0, scl=Pin(22), sda=Pin(23))
# sensor = LTR329(i2c)
# odm = DualWavelengthOD(sensor, vis_pin=19, ir_pin=16)
# V0 = odm.set_blank()
# while True:
#     print(odm.read_all())
#     time.sleep(1)





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
        sleep(0.01)

        self._write8(self.CONTROL_REG, 0x01)
        sleep(0.1)

        ### (Optional) adjust measurement/integration:
        # self._write8(self.MEAS_RATE_REG, 0x14)  # 100ms int, 500ms rate
        # sleep(0.1)

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
