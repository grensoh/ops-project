from machine import Pin # import hardware libraries
import time # used for delays in programs

led = Pin(25, Pin.OUT) # define an led object

while True: # forever
    led.value(1) # turn on led
    time.sleep(1) # wait 1 second
    led.value(0) # turn off led
    time.sleep(1) # wait 1 second
