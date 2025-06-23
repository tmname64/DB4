from machine import Pin, PWM
import time

class LED:
    def __init__(self, pin_num: int, active_high: bool = True) -> None:
        self._pin = Pin(pin_num, Pin.OUT)
        self._active_high = active_high
        # ensure LED is off at start
        self.off()

    def on(self) -> None:
        self._pin.value(1 if self._active_high else 0)

    def off(self) -> None:
        self._pin.value(0 if self._active_high else 1)

    def toggle(self) -> None:
        self._pin.value(not self._pin.value())

    def blink(self, on_time: float, off_time: float = None, times: int = 1) -> None:
        off_time = off_time if off_time is not None else on_time
        for _ in range(times):
            self.on()
            time.sleep(on_time)
            self.off()
            time.sleep(off_time)


class RGB_LED:
    def __init__ (self, redPin=19, greenPin=16, bluePin=18)-> None:
        self.Red = PWM(Pin(redPin))
        self.Green = PWM(Pin(greenPin))
        self.Blue = PWM(Pin(bluePin))

    def turn_off_led(self):
        self.Red.duty(0)
        self.Green.duty(0)
        self.Blue.duty(0)
    
    def turn_on_led(self):
        self.Red.duty(1000)
        self.Green.duty(0)
        self.Blue.duty(1000)




class UV_LED:
    def __init__(self, pin_num, freq=1000):
        self.uv = PWM(Pin(pin_num)) # PWM for brightness control (optional)
        self.uv.freq(freq)

    def off(self):
        self.uv.duty(0)

    def on(self, intensity=512):
        self.uv.duty(intensity) # duty: 0–1023 on ESP32

# ### Example Usage ###
# uv_led = UV_LED(pin_num=17)
# uv_led.on(700)   # turn UV LED on at ~70% power
# # …
# uv_led.off()
