from machine import I2C, Pin
from time import sleep
import time



class DualWavelengthOD:
    """
    - sensor: instance with read_lux() -> (ch0, ch1)
    - vis_pin: GPIO pin for visible LED
    - ir_pin: GPIO pin for IR LED
    - blank_reference: reference intensity V0 (set after measuring blank)
    """
    def __init__(self, sensor, vis_pin, ir_pin, blank_reference=None, pulse_ms=50):
        self.sensor = sensor
        self.vis_led = Pin(vis_pin, Pin.OUT)
        self.ir_led  = Pin(ir_pin,  Pin.OUT)
        self.blank_reference = blank_reference
        self.pulse_ms = pulse_ms
        # ensure LEDs off
        self.vis_led.value(0)
        self.ir_led.value(0)

    def _pulse_led(self, led_pin):
        # turn on, wait, read, then off
        led_pin.value(1)
        time.sleep_ms(self.pulse_ms)
        ch0, ch1 = self.sensor.read_lux()
        led_pin.value(0)
        return ch0, ch1

    def measure_intensities(self):
        """
        Performs a dual-wavelength cycle:
        - visible LED: returns total (T_vis)
        - IR LED: returns scatter (S_ir)
        Returns (T_vis, S_ir)
        """
        # measure with visible LED
        t_vis_ch0, t_vis_ch1 = self._pulse_led(self.vis_led)
        # measure with IR LED
        s_ir_ch0, s_ir_ch1 = self._pulse_led(self.ir_led)
        # For visible correction we use:
        # T_vis = total reading from visible LED channel (we use ch0)
        # S_ir  = IR-only scattering from IR LED (we use ch1)
        return t_vis_ch0, s_ir_ch1

    def set_blank(self, samples=3):
        """
        Measure blank reference intensity V0 (mean over samples).
        Must be called before compute_od.
        """
        vals = []
        for _ in range(samples):
            t_vis, s_ir = self.measure_intensities()
            vals.append(t_vis - s_ir)
            time.sleep_ms(100)
        self.blank_reference = sum(vals) / len(vals)
        return self.blank_reference

    def compute_od(self):
        """
        OD = -log10( (T_vis - S_ir) / V0 )
        Requires blank_reference set.
        """
        if self.blank_reference is None:
            raise RuntimeError("Blank reference not set. Call set_blank() first.")
        t_vis, s_ir = self.measure_intensities()
        V = t_vis - s_ir
        if V <= 0 or self.blank_reference <= 0:
            return float('inf')
        return -math.log10(V / self.blank_reference)

    def read_all(self):
        """
        Returns a dict with:
           'T_vis', 'S_ir', 'V', 'OD'
        """
        t_vis, s_ir = self.measure_intensities()
        V = t_vis - s_ir
        od = None
        if self.blank_reference is not None and V > 0:
            od = -math.log10(V / self.blank_reference)
        return {
            'T_vis': t_vis,
            'S_ir': s_ir,
            'V_corrected': V,
            'OD': od
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
