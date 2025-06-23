from machine import Pin, ADC
from math import log
import machine

from config import adc_V_lookup


# Constants
NOM_RES       = 10000       # Thermistor nominal resistance at 25 °C
SER_RES       = 9820        # Series resistor value
TEMP_NOM      = 23         # Nominal temperature (°C) for R0
THERM_B_COEFF = 3950        # Beta coefficient
NUM_SAMPLES   = 50          # Number of ADC samples to average
ADC_MAX       = 1023        # 10-bit ADC max count
ADC_VREF      = 3.3        # Reference voltage corresponding to ADC_MAX

class TemperatureSensor:
    def __init__(self, pin_number) -> None:
        self.adc = ADC(Pin(pin_number))
        self.adc.atten(ADC.ATTN_11DB)
        self.adc.width(ADC.WIDTH_10BIT)
        
    def read_temp(self):
        # 1) Take multiple readings and average
        total = 0
        for _ in range(NUM_SAMPLES):
            total += self.adc.read()
        avg_count = total / NUM_SAMPLES

        # 2) Convert ADC count → voltage (you can also use your adc_V_lookup here)
        #    If you prefer lookup: V_meas = adc_V_lookup[round(avg_count)]
        V_meas = (avg_count / ADC_MAX) * ADC_VREF

        # 3) Compute thermistor resistance from the divider equation
        R_th = SER_RES * V_meas / (ADC_VREF - V_meas)

        # 4) Steinhart–Hart (Beta) equation, corrected sign:
        T0    = TEMP_NOM + 273.15
        invT  = (1.0 / T0) + (1.0 / THERM_B_COEFF) * log(R_th / NOM_RES)
        tempK = 1.0 / invT
        tempC = tempK - 273.15

        return tempC
