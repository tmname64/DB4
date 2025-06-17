from machine import Pin, ADC
import time
import math

class LightSensor:
    def __init__(self, adc_pin=22, light_pin=23, reference=1620):
        self.adc = ADC(Pin(adc_pin))
        self.adc.atten(ADC.ATTN_11DB)      # Full voltage range (0-3.3V)
        self.adc.width(ADC.WIDTH_12BIT)    # 12-bit resolution: 0–4095
        self.reference = reference         # Light level with clear solution
        self.light = Pin(light_pin, Pin.OUT)

    def read_intensity(self, samples=100, delay=0.1):
        """Turn off light, wait, read intensity average over `samples` readings."""
        self.light.value(1)  # Turn OFF LED
        time.sleep(delay)
        values = [self.adc.read() for _ in range(samples)]
        self.light.value(0)  # Turn ON LED again
        return sum(values) / len(values)

    def compute_optical_density(self):
        """Compute OD using Beer-Lambert approximation."""
        intensity = self.read_intensity()
        if intensity <= 0:
            return float('inf')  # Avoid log(0)
        return -math.log10(intensity / self.reference)

    def compute_concentration(self, od):
        """Convert OD to estimated cell concentration using fitted model."""
        return 1048835.78 * od + 7370.76

    def read_all(self):
        """Convenience function to return everything."""
        intensity = self.read_intensity()
        od = self.compute_optical_density()
        conc = self.compute_concentration(od)
        return {
            "intensity": intensity,
            "OD": od,
            "concentration": conc
        }
