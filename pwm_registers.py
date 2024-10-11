from machine import Pin, PWM, ADC, mem32
import uctypes
import array
import math
import utime
import sys

voltage_resolution = 16
nVoltages = (2**voltage_resolution) - 1

adc_pin = 26
pwm_pin = 16
dma_channel = 0


###########################
######  PWM Registers #####
###########################
PWM_BASE          = 0x40050000
PWM_CHANNEL_WIDTH = 0x14

pwm_slice = (pwm_pin >> 1) & 7
pwm_channel_offset = pwm_slice * PWM_CHANNEL_WIDTH

PWM_CSR = PWM_BASE | (0x00 + pwm_channel_offset) # Control and status
PWM_DIV = PWM_BASE | (0x04 + pwm_channel_offset) # Clock divider
PWM_CTR = PWM_BASE | (0x08 + pwm_channel_offset) # PWM counter
PWM_CC  = PWM_BASE | (0x0c + pwm_channel_offset) # Counter compare
PWM_TOP = PWM_BASE | (0x10 + pwm_channel_offset) # Counter wrap

PWM_EN  = PWM_BASE + 0xa0

#############################
######  PAD Registers  ######
#############################
PAD_BASE            = 0x4001c000
PAD_CHANNEL_WIDTH   = 0x04
PAD_PIN_COUNT       = 30

adc_pad_channel_offset = (adc_pin + 1) * PAD_CHANNEL_WIDTH
pwm_pad_channel_offset = (pwm_pin + 1) * PAD_CHANNEL_WIDTH

ADC_PAD = PAD_BASE | (0x00 + adc_pad_channel_offset)
PWM_PAD = PAD_BASE | (0x00 + pwm_pad_channel_offset)

# Enable output disable PAD_OD(bit 7)
mem32[ADC_PAD] |= (1 & 0x1) << 7
# Do not enable input enable PAD_IE(bit 6)
mem32[ADC_PAD] |= (0 & 0x1) << 6
# Set PAD_DRIVE(bits 4:5) to 2mA
mem32[ADC_PAD] |= (0x00 & 0x03) << 4
# Do not enable pull up PAD_PUE(bit 3)
mem32[ADC_PAD] |= (0 & 0x1) << 3
# Do not enable pull down PAD_PDE(bit 2)
mem32[ADC_PAD] |= (0 & 0x1) << 2
# Do not enable Schmitt trigger PAD_SCHMITT(bit 1)
mem32[ADC_PAD] |= (0 & 0x1) << 1
# Slew rate slow PAD_SLEWFAST(bit 0)
mem32[ADC_PAD] |= (0 & 0x1) << 0 
#print("ADC PAD Registers initialized")

# Do not enable output disable PAD_OD(bit 7)
mem32[PWM_PAD] |= (0 & 0x1) << 7
# Do not enable input enable PAD_IE(bit 6)
mem32[PWM_PAD] |= (1 & 0x1) << 6
# Set PAD_DRIVE(bits 4:5) to 2mA
mem32[PWM_PAD] |= (0x00 & 0x03) << 4
# Do not enable pull up PAD_PUE(bit 3)
mem32[PWM_PAD] |= (0 & 0x1) << 3
# Do not enable pull down PAD_PDE(bit 2)
mem32[PWM_PAD] |= (0 & 0x1) << 2
# Do not enable Schmitt trigger PAD_SCHMITT(bit 1)
mem32[PWM_PAD] |= (0 & 0x1) << 1
# Slew rate fast PAD_SLEWFAST(bit 0)
mem32[PWM_PAD] |= (1 & 0x1) << 0 
#print("PWM PAD Registers initialized")

#############################
######  GPIO Registers  #####
#############################
GPIO_BASE       = 0x40014000
GPIO_CHANNEL_WIDTH = 0x08
GPIO_PIN_COUNT  = 30

adc_gpio_channel_offset = adc_pin * GPIO_CHANNEL_WIDTH
pwm_gpio_channel_offset = pwm_pin * GPIO_CHANNEL_WIDTH

ADC_GPIO_STATUS = GPIO_BASE | (0x00 + adc_gpio_channel_offset)
ADC_GPIO_CTRL   = GPIO_BASE | (0x04 + adc_gpio_channel_offset)

PWM_GPIO_STATUS = GPIO_BASE | (0x00 + pwm_gpio_channel_offset)
PWM_GPIO_CTRL   = GPIO_BASE | (0x04 + pwm_gpio_channel_offset)

#print(hex(GPIO_CTRL))
mem32[ADC_GPIO_CTRL] = 0x1f # GPIO_FUNC_NULL, gpio_set_function()
mem32[PWM_GPIO_CTRL] = 0x4  # gpio_set_function(pin, GPIO_FUNC_PWM)
#print("GPIO Registers initialized")

#############################
######  ADC Registers  ######
#############################
ADC_BASE      = 0x4004c000
ADC_CS        = ADC_BASE + 0x00
ADC_RESULT    = ADC_BASE + 0x04
ADC_FCS       = ADC_BASE + 0x08
ADC_FIFO      = ADC_BASE + 0x0c
ADC_DIV       = ADC_BASE + 0x10

#############################
######  DMA Registers  ######
#############################
DMA_BASE           = 0x50000000
DMA_CHANNEL_WIDTH  = 0x40
DMA_CHANNEL_COUNT  = 12

dma_channel_offset = dma_channel * DMA_CHANNEL_WIDTH

DMA_READ_ADDR = DMA_BASE | (0x00 + dma_channel_offset)
DMA_WRITE_ADDR = DMA_BASE | (0x04 + dma_channel_offset)
DMA_TRANS_COUNT = DMA_BASE | (0x08 + dma_channel_offset)
DMA_CTRL_TRIG = DMA_BASE | (0x0c + dma_channel_offset)
# Trigger register: writing a nonzero value will
# reload the channel counter and start the channel
DMA_AL1_CTRL = DMA_BASE | (0x10 + dma_channel_offset) 

# 16 bit mask that will abort an in-progress transfer sequence
# on those channels
DMA_CHAN_ABORT = DMA_BASE | 0x444


##############################


#pwm = PWM(Pin(pwm_pin, Pin.OUT))
#pwm.freq(20_000)
#pwm.duty_u16(int(0.2*nVoltages))

#for i in range(10):
#    a = (mem32[PWM_CTR] >> 0) & 0xffff
#    print(int(a))

# Disable all PWMs
mem32[PWM_EN] = 0

# Default options/reset CSR and counters
# (no PH_ADV or PH_RET, Free running, No inversion, trailing edge)
mem32[PWM_CSR] = 0
mem32[PWM_DIV] = 0
mem32[PWM_CTR] |= (0x0 & 0xffff) << 0 # Bits 0:15

# set_clkdiv_int_frac(div, 0)
pwm_div_int  = 1
pwm_div_frac = 0
mem32[PWM_DIV] |= (pwm_div_int & 0xff) << 4 #  Bits 4:11
mem32[PWM_DIV] |= (pwm_div_frac & 0xf) << 0 #  Bits 0:3

# set_wrap(wrap)
pwm_wrap = 0xffff
mem32[PWM_TOP] |= (pwm_wrap & 0xffff) << 0 # Bits 0:15 for channel A

# Set Counter compare
pwm_level = int(0.2 * nVoltages)
mem32[PWM_CC] |= (pwm_level & 0xffff) << 0 # Bits 0:15 for channel A

# Enable PWMs simultaneously
mask = 0
for i in range(1):
    mask |= 1 << i
mem32[PWM_EN] = mask



mem32[ADC_CS] = 0 # CS_REG = 0
mem32[ADC_FCS] = 0 # FCS_REG = 0
mem32[ADC_CS] |= (1 & 0x1) << 0  # ADC_CS_EN = 1     
# Start sampling at ADC channel 2
mem32[ADC_CS] |= (2 & 0x07) << 12 # ADC_CS_AINSEL (bits 12:14) = {0, 1, 2, 3, 4}
# Do a round robin sampling of only one channel (ADC channel 2)
mem32[ADC_CS] |= (0x4 & 0x1f) << 16  # ADC_CS_RROBIN (bits 16:20) = 0100

"""
# Checks if ADC is working
mem32[ADC_CS] |= (1 & 0x1) << 2     # ADC_CS_START_ONCE (bit 2) = 1
print(mem32[ADC_RESULT])
"""

# adc_fifo_setup()
# Allow conversion result to be written into FIFO (instead of RESULT)
mem32[ADC_FCS] |= (1 & 0x1) << 0 # ADC_FCS_EN (bit 0) = 1
# Assert a DMA request when FIFO contains data (for transfer into a buffer)
mem32[ADC_FCS] |= (1 & 0x1) << 3 # ADC_FCS_DREQ_EN (bit 3) = 1
# Assert DREQ/IRQ when there is n bytes of data in the FIFO
mem32[ADC_FCS] |= (0x1 & 0x0f) << 24 # ADC_FCS_THRESH (bits 24:27) = 1
# Do not include error bits in conversion result
mem32[ADC_FCS] |= (0 & 0x1) << 2 # ADC_FCS_ERR (bit 2) = 0
# Do not right shift FIFO results
mem32[ADC_FCS] |= (0 & 0x1) << 1 # ADC_FCS_SHIFT (bit 1) = 0
# Clear "underflowed FIFO" flag
mem32[ADC_FCS] |= (1 & 0x1) << 10 # ADC_FCS_UNDER (bit 10) = 1
# Clear "overflowed FIFO" flag
mem32[ADC_FCS] |= (1 & 0x1) << 11 # ADC_FCS_OVER (bit 10) = 1

# adc_fifo_drain()
x = 0
adc_fcs_level = (mem32[ADC_FCS] >> 16) & 0x0F
while adc_fcs_level:
    x = mem32[ADC_FIFO]
    adc_fcs_level = (mem32[ADC_FCS] >> 16) & 0x0F


NSAMPLES = 10
RATE = 100_000
adc_buff = array.array('H', (0 for _ in range(NSAMPLES)))

# adc_set_clk_div()
mem32[ADC_DIV] = (48_000_000 // RATE - 1) << 8

# dma_channel_config()
# Abort operations of all 16 channels
mem32[DMA_CHAN_ABORT] = 0xffff
# Reset all TRIG registers (EN, SIZE, ...)
mem32[DMA_CTRL_TRIG] = 0
# Disable CHAIN_TO by setting to the same channel
mem32[DMA_CTRL_TRIG] |= (dma_channel & 0x4) << 11 # CHAIN_TO(bits 11:14)
# Increment write address with each transfer
mem32[DMA_CTRL_TRIG] |= (1 & 0x1) << 5 # INCR_WRITE(bit 5)
# Do not raise IRQ at the end of every transfer
mem32[DMA_CTRL_TRIG] |= (1 & 0x1) << 21 # IRQ_QUIET(bit 21)
# Select data request DREQ_ADC(36) as transfer request (3f=111111)
mem32[DMA_CTRL_TRIG] |= (36 & 0x3f) << 15 # TREQ_SEL(bits 15:20, 6 bits)
# Set DATA_SIZE to SIZE_HALFWORD(0x1, 2 bytes)
mem32[DMA_CTRL_TRIG] |= (0x1 & 0x3) << 2 # DATA_SIZE(bits 2:3)
# Read from the ADC_FIFO address
mem32[DMA_READ_ADDR] = ADC_FIFO
# Write to the (established) adc buffer array
mem32[DMA_WRITE_ADDR] = uctypes.addressof(adc_buff)
# Get this many (read, write) sample transfers
mem32[DMA_TRANS_COUNT] = NSAMPLES


# dma_channel_start()
mem32[DMA_CTRL_TRIG] |= (1 & 0x1) << 0 # CTRL_TRIG_EN(bit 0) = 1

# adc_run(true)
mem32[ADC_CS] |= (1 & 0x1) << 3 # ADC_CS_START_MANY(bit 3) = 1

# ADC running
adc_running = (mem32[DMA_CTRL_TRIG] >> 24) & 0x1
while adc_running:
    utime.sleep_ms(10)
    adc_running = (mem32[DMA_CTRL_TRIG] >> 24) & 0x1
# adc_run(false)
mem32[ADC_CS] |= (0 & 0x1) << 3 # ADC_CS_START_MANY(bit 3) = 1

vals = [f"{val}" for val in adc_buff]
print(vals)