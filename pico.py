from machine import Pin, PWM
import math
import time
import sys

HS = [ PWM(Pin(i)) for i in [0,2,4] ]
LS = [ Pin(i, Pin.OUT) for i in [1,3,5] ]

f = 80000
for i in HS:
    i.freq(f)

hsRef = PWM(Pin(6)) # Scope reference
hsRef.freq(f)

voltageResolution = 16 # bits
nVoltages = (2**voltageResolution) - 1
dutyCycleLimiter = 0.3

sinusoidPeriod = 500 # milliseconds
angularFrequency = 2 * math.pi / (sinusoidPeriod)

def Sinusoid(t, phase, limit=dutyCycleLimiter, omega=angularFrequency, p2p=nVoltages):
    amplitude = p2p / 2
    angle = omega * t + (phase * (2 * math.pi / 360))
    value = math.sin(angle)
    duty = amplitude * value
    return int(limit * duty)

def SetHighSide(phase, pwm):
    phase.duty_u16(pwm)
    return

def SetLowSide(phase, value):
    if value:
        phase.high()
        return
    else:
        phase.low()
        return

def Step(highSide, lowSide):
    if highSide == 0:
        if lowSide == 1:
            return 1
        else:
            return 2
    elif highSide == 1:
        if lowSide == 0:
            return 5
        else:
            return 6
    else:
        if lowSide == 0:
            return 4
        else:
            return 3
    
t0 = time.ticks_ms()
hsRef.duty_u16(int(nVoltages/2)) 

phaseH = 0
phaseL = 0
step = 0
while True:
    dt = time.ticks_diff(time.ticks_ms(), t0)
    dt %= sinusoidPeriod
    DUTY = [Sinusoid(dt, 0), Sinusoid(dt, 120), Sinusoid(dt, 240)]
    maxI = DUTY.index(max(DUTY))
    minI = DUTY.index(min(DUTY))
    
    if maxI != phaseH:
        SetHighSide(HS[phaseH], 0)
        phaseH = maxI
        #step = Step(phaseH, phaseL)
        #print(f"{dt}: {phaseH} -> {phaseL}, step {step}")
    if minI != phaseL:
        SetLowSide(LS[phaseL], 0)
        phaseL = minI
        SetLowSide(LS[phaseL], 1)
        #step = Step(phaseH, phaseL)
        #print(f"{dt}: {phaseH} -> {phaseL}, step {step}")
    
    
    SetHighSide(HS[phaseH], DUTY[phaseH])

