from machine import Pin, PWM
import math
import time

hsa = PWM(Pin(1))
hsb = PWM(Pin(2))
hsc = PWM(Pin(3))

lsa = Pin(4, Pin.OUT)
lsb = Pin(5, Pin.OUT)
lsc = Pin(6, Pin.OUT)

PWMfreq = 20000 # Hz
sinusoidPeriod = 250 # ms
sinusoidfreq = 1000 / sinusoidPeriod # Hz
angularfreq = 2 * math.pi * sinusoidfreq

tB = 90 * (2 * math.pi / 360) / angularfreq
tC = 210 * (2 * math.pi / 360) / angularfreq
tA = 330 * (2 * math.pi / 360) / angularfreq

timeResolution = 10 # bits
nTime = (2**timeResolution) - 1
dt = 1 / (sinusoidfreq * nTime)

voltageResolution = 16 # bits
nVoltages = (2**voltageResolution) - 1
maxDutyCycle = 0.1

hsa.freq(PWMfreq)
hsb.freq(PWMfreq)
hsc.freq(PWMfreq)

LS = [0,0,0] # index for transitioning low side {A, B, C}
for i in range(nTime):
    val = i * dt
    if val <= tB:
        LS[1] = i
    if val <= tC:
        LS[2] = i
    if val <= tA:
        LS[0] = i
        
def Sinusoid(i, ntime, p2p, maxDutyCycle=maxDutyCycle):
    amplitude = p2p / 2
    val = math.sin(2 * math.pi * i / ntime)
    duty = amplitude * (val + 1)
    return int(maxDutyCycle * duty)

def SetHighSide(i, N, table, A, B, C):
    pwmA = table[i]
    pwmB = table[ (i + int(N / 3)) % N ]
    pwmC = table[ (i + 2 * int(N / 3)) % N ]
    
    A.duty_u16(pwmA)
    B.duty_u16(pwmB)
    C.duty_u16(pwmC)
    return

def SetLowSide(i, A, B, C, table):
    if i == table[1]: # Transition from LSB to LSC
        B.low()
        C.high()
        return
    if i == table[2]: # Transition from LSC to LSA
        C.low()
        A.high()
        return
    if i == table[0]: # Transition from LSA to LSB
        A.low()
        B.high()
        return

PWM = [ Sinusoid(i, dt, nVoltages) for i in range(nTime) ]

i = 0
lsb.high()
while True:
    if i in LS:
        SetLowSide(i, lsa, lsb, lsc, LS)
    SetHighSide(i, nTime, PWM, hsa, hsb, hsc)
    i = (i + 1) % nTime
    time.sleep(dt)
