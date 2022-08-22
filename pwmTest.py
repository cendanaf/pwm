pwmPin = [4, 5, 6]
enPin = [9, 10, 11]

PWMResolution = 16
MAX_DUTY_CYCLE = 2 ** PWMResolution
duty = int( 0.4 * MAX_DUTY_CYCLE )

pwm = []
for i in pwmPin:
    pwm.append(PWM(Pin(i)))
    
en = []
for i in enPin:
    en.append(Pin(i, Pin.OUT))