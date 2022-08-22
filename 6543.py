import time
from machine import Pin, PWM

buttonPin = 0
ledPin = 25
pwmPin = [2, 3, 4]
enPin = [6, 7, 8]

debounceTime = 0.005
lastDebounceTime = 0

PWMFreq = 100000
PWMResolution = 16
MAX_DUTY_CYCLE = 2 ** PWMResolution
duty = int( 0.4 * MAX_DUTY_CYCLE )

button = Pin(buttonPin, Pin.IN, Pin.PULL_UP)
led = Pin(ledPin, Pin.OUT)
buttonState = False
ledState = False
transientState = False
state = False
step = 0

pwm = []
for i in pwmPin:
    p = PWM(Pin(i))
    p.freq(PWMFreq)
    pwm.append(p)
    
en = []
for i in enPin:
    en.append(Pin(i, Pin.OUT))
    
en[1].value(1)
en[0].value(1)
pwm[0].duty_u16(duty)

while True:
    buttonState = button.value()
    
    if(state != transientState):
        #print("Transient state")
        lastDebounceTime = time.time()
        transientState = state
        
    if((time.time() - lastDebounceTime) > debounceTime):
        if(state == True and buttonState == False):
            #print("Button pressed")
            #ledState = not ledState
            #led.value(ledState)
            print("Step " + str(step))
            
            if step == 0:
                en[2].value(0)
                pwm[2].duty_u16(0)
                
                en[0].value(1)
                pwm[0].duty_u16(duty)
                
            elif step == 1:
                en[1].value(0)
                
                en[2].value(1)
            
            elif step == 2:
                en[0].value(0)
                pwm[0].duty_u16(0)
                
                en[1].value(1)
                pwm[1].duty_u16(duty)
                
            elif step == 3:
                en[2].value(0)
                
                en[0].value(1)
                
            elif step == 4:
                en[1].value(0)
                pwm[1].duty_u16(0)
                
                en[2].value(1)
                pwm[2].duty_u16(duty)
                
            else:
                en[0].value(0)
                en[1].value(1)
            
            step += 1
            step %= 6
            
        #elif(state == False and buttonState == True):
        #    print("Button released")
        
    
    state = buttonState