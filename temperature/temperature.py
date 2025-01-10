from machine import Pin, ADC
import time

adc = ADC(Pin(26)) # TMP36 analog pin

while True:
    value = adc.read_u16() # read pin value
    mv = 3300.0 * value / 65535 # scale data between 0V and 3.3 V
    temp = (mv - 500) / 10 # convert data following datasheet
    print(temp)
    time.sleep(0.4)

# watch out the pin value
