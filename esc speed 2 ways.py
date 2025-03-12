import time
from machine import Pin, PWM

# 
pwm_pin = PWM(Pin(1))  # Remplacez 15 par le numéro de la broche souhaitée
pwm_pin.freq(50)       # PWM signal frequency (50 Hz)

# PWM 20ms period: u16=65535 = 20ms = 20000us
# PWM should vary between 1000us on-time and 2000us on-time, with 0 (center) being at 1500us on-time.
# But there is a center zone between 1450 and 1500 us where motor won't spin.
#
# servo 0 (center) = PWM 1500 us ON time  = 1500/20000 *100 percent  = 7.5 %
#     full forward = PWM 2000 us ON time  = 2000                     = 10 %
#     ull backward = PWM 1000 us ON time  = 1000                     = 5 %

# Fonction pour définir le rapport cyclique (duty cycle)
def set_duty_cycle(pwm, duty):
    d = int(duty / 100 * 65535)
    # Conversion en microsecondes pour une plage de 16 bits
    print(f"d={d}")
    pwm.duty_u16(d)

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
        time.sleep(delay)
    time.sleep(2)
    set_speed(-max)
    time.sleep(5)
    set_speed(0)
    time.sleep(5)
    
    

"""time.sleep(1)
max = 100
delay = 0.0001
while True :
    print( "FORWARD incr to {max}")
    for i in range (0, max, +1) :
        set_speed(i)
        time.sleep(delay)
    time.sleep(5)
    for i in range (0,-max,-1) :
        set_speed(i)
        time.sleep(delay)
    time.sleep(5)"""

"""max = 80
delay = 0.05
print("FORWARD incr to {max}")
for i in range(0, max, +1):
    set_speed(i)
    time.sleep(delay)
print("FORWARD decr")
for i in range(max, 0, -1):
    set_speed(i)
    time.sleep(delay)
# Reverse
print("REVERSE incr to {max}")
for i in range(0, -max, -1):
    set_speed(i)
    time.sleep(delay)
print("REVERSE decr")
for i in range(-max, 0, +1):
    set_speed(i)
    time.sleep(delay)

set_speed(0)
"""
"""
while True:
   pass
   # Accélération
    i = 60
    while i < 75:
        print(f"Duty cycle: {i}%")
        set_duty_cycle(pwm_pin, i)
        time.sleep(0.05)
        i += 0.02

    # Décélération
    while i > 55:
        print(f"Duty cycle: {i}%")
        set_duty_cycle(pwm_pin, i)
        time.sleep(0.05)
        i -= 0.05
"""