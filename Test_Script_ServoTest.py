import time
from machine import Pin, PWM

servo = PWM(Pin(16))
servo.freq(50)

#Sets the angle values on each end
MIN_ANGLE = 0
MAX_ANGLE = 180

#Custom pulse values
MIN_US = 750   #Pulse width for 0 degrees
MAX_US = 2500  #Pulse width for 180 degrees

def set_angle_precise(angle):
    #Constrain input to safe boundaries
    if angle < MIN_ANGLE: angle = MIN_ANGLE
    if angle > MAX_ANGLE: angle = MAX_ANGLE
    
    #Maps the minimum and maximum angles
    pulse_us = MIN_US + ((angle - MIN_ANGLE) * (MAX_US - MIN_US) / (MAX_ANGLE - MIN_ANGLE))
    
    #Convert microseconds to 16-bit duty cycle
    duty = int((pulse_us / 20000) * 65535)
    servo.duty_u16(duty)

#Tests different angles
try:
    angles_to_test = [0, 45, 90, 135, 180]
    
    for a in angles_to_test:
        print(f"Moving to exactly {a} degrees")
        set_angle_precise(a)
        time.sleep(1.5)
        
finally:
    servo.deinit()
    print("PWM turned off safely.")

