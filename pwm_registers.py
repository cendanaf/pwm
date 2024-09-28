from machine import Pin, PWM, mem32
import math
import time
import sys

voltage_resolution = 16
nVoltages = (2**voltage_resolution) - 1

HS_pins = [16, 18, 20]
LS_pins = [17, 19, 21]
HS = [ PWM(Pin(i)) for i in HS_pins ]
LS = [ Pin(i, Pin.OUT) for i in LS_pins ]

pwm_slice_num = [ (pin >> 1) & 7 for pin in HS_pins ]


PWMfreq = 20_000
for pwm in HS:
    pwm.freq(PWMfreq)

HS[0].duty_u16(int(0.2 * nVoltages))
HS[1].duty_u16(int(0.2 * nVoltages))
HS[2].duty_u16(int(0.2 * nVoltages))

# RP2040 datasheet page 533
PWM_BASE = 0x40050000
PWM_EN = PWM_BASE + 0xa0

# Channels 16, 18, 20 are mapped to 0A, 1A, 2A
# So we need CH0, CH1, CH2
CH0_CSR = PWM_BASE + 0x00 # Control and status register
CH0_DIV = PWM_BASE + 0x04 # Clock divider
CH0_CTR = PWM_BASE + 0x08 # Counter
CH0_CC  = PWM_BASE + 0x0c # Counter compare
CH0_TOP = PWM_BASE + 0x10 # Counter wrap

CH1_CSR = PWM_BASE + 0x14 # Control and status register
CH1_DIV = PWM_BASE + 0x18 # Clock divider
CH1_CTR = PWM_BASE + 0x1c # Counter
CH1_CC  = PWM_BASE + 0x20 # Counter compare
CH1_TOP = PWM_BASE + 0x24 # Counter wrap

CH2_CSR = PWM_BASE + 0x28 # Control and status register
CH2_DIV = PWM_BASE + 0x2c # Clock divider
CH2_CTR = PWM_BASE + 0x30 # Counter
CH2_CC  = PWM_BASE + 0x34 # Counter compare
CH2_TOP = PWM_BASE + 0x38 # Counter wrap

# Disable all PWMs
mem32[PWM_EN] = 0

# Reset Counters
mem32[CH0_CTR] = 0
mem32[CH1_CTR] = 0 
mem32[CH2_CTR] = 0

#PWM_DIV = 125
#mem32[CH0_DIV] = PWM_DIV
#mem32[CH1_DIV] = PWM_DIV
#mem32[CH1_DIV] = PWM_DIV

#PWM_WRAP = 49
#mem32[CH0_TOP] = PWM_WRAP
#mem32[CH1_TOP] = PWM_WRAP
#mem32[CH2_TOP] = PWM_WRAP

#PWM_LEVEL = int(0.2 * nVoltages)
#mem32[CH0_CC] = PWM_LEVEL
#mem32[CH1_CC] = PWM_LEVEL
#mem32[CH2_CC] = PWM_LEVEL

# Enable PWMs simultaneously
mask = 0
for i in pwm_slice_num:
    mask |= 1 << i
mem32[PWM_EN] = mask

#Enable PWMs sequentially
#mem32[CH0_CSR] = 1
#mem32[CH1_CSR] = 1
#mem32[CH2_CSR] = 1
