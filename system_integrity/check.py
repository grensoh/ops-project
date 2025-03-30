import gc
from machine import Pin
import time

led = Pin(25, Pin.OUT)

gc.collect()

mem_allocated = gc.mem_alloc()
mem_free = gc.mem_free()

print(f"Mémoire allouée : {mem_allocated} bytes")
print(f"Mémoire libre : {mem_free} bytes")

while True:
    led.value(1) # turn on led
    time.sleep(1) # wait 1 second
    led.value(0) # turn off led
    time.sleep(1) # wait 1 second
