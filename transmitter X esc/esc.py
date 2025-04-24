from machine import Pin
import time

pin_horaire = Pin(2, Pin.OUT)
pin_antihoraire = Pin(4, Pin.OUT)

def set_motor(direction):
    if direction == 1:
        pin_antihoraire.value(0)
        pin_horaire.value(1)
    elif direction == -1:
        pin_horaire.value(0)
        pin_antihoraire.value(1)
    elif direction == 0:
        pin_horaire.value(0)
        pin_antihoraire.value(0)
