import time
from machine import Pin, PWM

pwm_pin = PWM(Pin(1))  # pin used to control esc
pwm_pin.freq(50)       # PWM signal frequency (50 Hz)

def set_speed(speed):
    """Sets speed between -100 and 100 (%)"""

    # Accomodate center zone (for reverse, start at 1460 us)
    if speed > 0.0:
        middle = 1500000
    else:
        middle = 1460000

    range = 500000.0
    ns = middle + int((speed/100.0) * range)
    #print(f"us={ns/1000}")
    pwm_pin.duty_ns(ns)

def calibrate():
    print("Calibrating ESC")
    #set_duty_cycle(pwm_pin, 2.5)  # 3% duty cycle
    # Force ESC calibration by starting at 100%, go to -100% so ESC sees full range of input
    for i in range(100, -100, -1):
        set_speed(i)
        time.sleep(0.005)
    # Then back to 0% to complete arming
    for i in range(-100, 0, +1):
        set_speed(i)
        time.sleep(0.005)
    set_speed(0)
    time.sleep(1)
    print("Calibration DONE")

def arm():
    print("Arming ESC")
    for i in range(0, 25, +1):
        set_speed(i)
        time.sleep(0.005)
    # Then back to 0% to complete arming
    for i in range(25, 0, -1):
        set_speed(i)
        time.sleep(0.005)
    set_speed(0)
    time.sleep(1)
    #time.sleep(5)
    print("Arming DONE")

# Initialisation
print("Starting at 0%")
set_duty_cycle(pwm_pin, 0)

arm()

#Principal loop : accelerating -> max (2s) -> -max(5s) -> stop(5s)
time.sleep(3)
max = 100
delay = 0.02
while True :
    print("Forward increase to {max}")
    for i in range (0, max, +1) :
        set_speed(i)
    print("Forward decrease to 0")
    for i in range(max, 0, +1)
