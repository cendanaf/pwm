from machine import Pin, PWM, mem32
import math
import time
import sys

voltage_resolution = 16
nVoltages = (2**voltage_resolution) - 1
HS = [ PWM(Pin(i)) for i in [16,18,20] ]
LS = [ Pin(i, Pin.OUT) for i in [17,19,21] ]

PWMfreq = 20_000
for pwm in HS:
    pwm.freq(PWMfreq * 2)

# RP2040 datasheet page 533
PWM_BASE = 0x40050000
PWM_EN = PWM_BASE + 0xa0
PWM_INTR = PWM_BASE + 0xa4
CH0_CSR = PWM_BASE + 0x00

# Channels 16, 18, 20 are mapped to 0A, 1A, 2A
# So we need CH0, CH1, CH2
CH0_CSR = PWM_BASE + 0x00 # Control and status register
CH0_CTR = PWM_BASE + 0x08 # Counter
CH1_CSR = PWM_BASE + 0x14
CH1_CTR = PWM_BASE + 0x1c
CH2_CSR = PWM_BASE + 0x28
CH2_CTR = PWM_BASE + 0x30

# Disables all PWMs
mem32[PWM_EN] = 0

# Enable phase-correct modulation (p534)
mem32[CH0_CSR] = mem32[CH0_CSR] | 0x2
mem32[CH1_CSR] = mem32[CH1_CSR] | 0x2
mem32[CH2_CSR] = mem32[CH2_CSR] | 0x2

# Reset Counters
mem32[CH0_CTR] = 0
mem32[CH1_CTR] = 0
mem32[CH2_CTR] = 0

# Enable PWMs simultaneously
mem32[PWM_EN] = (1<<0) | (1<<7)

HS[0].duty_u16(int(0.2 * nVoltages))
HS[1].duty_u16(int(0.3 * nVoltages))
HS[2].duty_u16(int(0.4 * nVoltages))