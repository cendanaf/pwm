from machine import Pin, PWM
import math
import time
import sys

hsa = PWM(Pin(0))
lsa = Pin(1, Pin.OUT)

hsb = PWM(Pin(2))
lsb = Pin(3, Pin.OUT)

hsc = PWM(Pin(4))
lsc = Pin(5, Pin.OUT)

hsd = PWM(Pin(6)) # reference (for oscilloscope)

f = 20000
hsa.freq(f)
hsb.freq(f)
hsc.freq(f)

voltageResolution = 16 # bits
nVoltages = (2**voltageResolution) - 1
dutyCycleLimiter = 0.5
duty = int(nVoltages/2)

button = Pin(9, Pin.IN, Pin.PULL_DOWN)

sinusoidPeriod = 500 # milliseconds
angularFrequency = 2 * math.pi / (sinusoidPeriod)

tB = 90 / angularFrequency
tC = 210 / angularFrequency
tA = 330 / angularFrequency

def Sinusoid(t, phase, limit, omega=angularFrequency, p2p=maxDutyCycle):
    amplitude = p2p / 2
    angle = omega * t + phase
    value = math.sin(angle)
    duty = amplitude * (value + 1)
    return int(limit * duty)

def SetLowSide(dt, A=lsa, B=lsb, C=lsc, tA=tA, tB=tB, tC=tC):
    #print(dt)
    if A.value():
        #print("LSA high")
        if dt >= tA:
            A.low()
            B.high()
        return
    elif B.value():
        #print("LSB high")
        if dt >= tB:
            B.low()
            C.high()
            if dt > tC:
                C.low()
        return
    elif C.value():
        #print("LSC high")
        if dt >= tC:
            C.low()
            A.high()
            if dt > tA:
                A.low()
        return
    else:
        B.high()
        return
    
t0 = time.ticks_ms()
while True:
    dt = time.ticks_ms() - t0
    dt %= sinusoidPeriod
            
    SetLowSide(dt) 
    
    da = Sinusoid(dt, 0, dutyCycleLimiter)
    db = Sinusoid(dt, 120, dutyCycleLimiter)
    dc = Sinusoid(dt, 240, dutyCycleLimiter)
            
    hsa.duty_u16(da)
    hsb.duty_u16(db)
    hsc.duty_u16(dc)