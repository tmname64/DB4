from machine import Pin, PWM


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
