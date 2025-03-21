import array
import time
import machine
import math
from uctypes import addressof

# ====================================
# === Register write functions =======

def IOreg_read(reg_addr, bit, bit_mask):
    return (machine.mem32[reg_addr] >> bit) & bit_mask

def IOreg_write(reg_addr, data):
    machine.mem32[reg_addr] = data
#   
def IOreg_set(reg_addr, data):
    machine.mem32[reg_addr + 0x2000] = data
#
def IOreg_clr(reg_addr, data):
    machine.mem32[reg_addr + 0x3000] = data
#   
def IOreg_xor(reg_addr, data):
    machine.mem32[reg_addr + 0x1000] = data
    
# === IO =============================
IO_base   = 0x40014000
PADS_BASE = 0x4001c000

def GPIO_CTRL(chan_num):
    return chan_num*8 + 4 + IO_base

def GPIO_STATUS(chan_num):
    return chan_num*8 + IO_base

def GPIO_PAD_CTRL(pad_num):
    return (pad_num*4 + 4) + PADS_BASE

def GPIO2SliceNum(chan_num):
    return (chan_num >> 1) & 7
    
# === PWM ============================
# section 4.5
PWM_base =  0x40050000
PWM_INTR =  0xa4 + PWM_base
PWM_EN   =  0xa0 + PWM_base

def PWM_CSR(slice_num):
    return (0x14 * slice_num + PWM_base)

def PWM_divmode(mode):
    return (mode & 0x03)<<4

def PWM_DIV(slice_num):
    return (0x14 * slice_num + PWM_base + 0x04)

# the actual PWM 16-bit couonter
def PWM_CTR(slice_num):
    return (0x14 * slice_num + PWM_base + 0x08)

# counter compare 31:16 B, 15:0 A
def PWM_CC(slice_num):
    return (0x14 * slice_num + PWM_base + 0x0c)

# counter wrap 16 bit value
def PWM_TOP(slice_num):
    return (0x14 * slice_num + PWM_base + 0x10)



# ====================================
# PWM setup 
PWM_FN = 4
mask = 0
free_run = 0

IOreg_write(GPIO_CTRL(0), PWM_FN )# pwm function slice 0A
IOreg_write(GPIO_CTRL(4), PWM_FN ) # pwm function slice 2A


# channel 0
wrapVal = 6249
cc = int(0.2 * wrapVal)
IOreg_write(PWM_DIV(0), 1<<4) 
IOreg_write(PWM_TOP(0), wrapVal)
IOreg_write(PWM_CC(0), cc) 
IOreg_write(PWM_CSR(0), PWM_divmode(free_run))
mask |= 1 << GPIO2SliceNum(0)

# channel 4
wrapVal = 624
cc = int(wrapVal/2)
IOreg_write(PWM_DIV(GPIO2SliceNum(4)), 1<<4) 
IOreg_write(PWM_TOP(GPIO2SliceNum(4)), wrapVal) 
IOreg_write(PWM_CC(GPIO2SliceNum(4)), cc) 
IOreg_write(PWM_CSR(GPIO2SliceNum(4)), PWM_divmode(free_run))
mask |= 1 << GPIO2SliceNum(4)

IOreg_write(PWM_EN, mask) # enable PWM simultaneously

# === ADC ============================
ADC_BASE    = 0x4004c000
ADC_CS      = 0x00 + ADC_BASE
ADC_RESULT  = 0x04 + ADC_BASE
ADC_FIFO_CS = 0x08 + ADC_BASE
ADC_FIFO    = 0x0c + ADC_BASE
ADC_DIV     = 0x10 + ADC_BASE


def ADC_RROBIN(channels):
    return (channels & 0x1f)<<16

def ADC_AINSEL(channel):
    return (channel & 0x07)<<12

def ADC_READY(ready):
    return (ADC_CS & (1<<8))

ADC_START_MANY = (1<<3)
ADC_START_ONCE = (1<<2)
ADC_EN = 1 # Power on ADC and enable its clock.

# === bits in ADC_FIFO CS
def ADC_THRESH(fifo_level):
    return (fifo_level & 0x0f)<<24

ADC_DREQ_EN = (1<<3)

# enable fifo
ADC_FIFO_EN = 1

def ADC_Div_int(int_part):
    return (int_part & 0xffff)<<8
def ADC_Div_frac(frac_part):
    return (frac_part & 0xff)

def adc_fifo_drain(p=False):
    while ((machine.mem32[ADC_FIFO_CS]>>16)&(0xf)):
        x = machine.mem32[ADC_FIFO]
        if(p):
            print(x)
    


#==== ADC Setup ============================

# Output disable, input disable, PUE and PDE disable
IOreg_write(GPIO_PAD_CTRL(26), 0b10000000)
#IOreg_write(GPIO_PAD_CTRL(27), 0b10000000)
#IOreg_write(GPIO_PAD_CTRL(28), 0b10000000)
# Pacing timer
IOreg_write(ADC_DIV, ADC_Div_int(96) | ADC_Div_frac(0))
# ADC control and status
print(bin(machine.mem32[ADC_CS]))
IOreg_write(ADC_CS, (ADC_RROBIN(0x7) |
                        ADC_AINSEL(0) |
                        #ADC_START_MANY |
                        ADC_EN))
print(bin(machine.mem32[ADC_CS]))
"""
# Sanity check
# Ensures ADC is functioning (on one shot mode)
for i in range(3):
    machine.mem32[ADC_CS] |= ADC_START_ONCE
    val = machine.mem32[ADC_RESULT]
    print(val)
    time.sleep(1)
"""

adc_fifo_drain(1)

IOreg_write(ADC_FIFO_CS, ADC_THRESH(1) |
                        ADC_DREQ_EN |
                        ADC_FIFO_EN )



#==== DMA ============================
# register adddresses for DMA
# datasheet page 102
DMA_base = 0x50000000
DMA_multi_chan_enable = DMA_base + 0x430
# ===================================
# === DMA channel control registers
# valid channels: 0 to 11
def DMA_RD_ADDR(ch_num):
    return (0x40*ch_num + DMA_base)
def DMA_WR_ADDR(ch_num):
    return (0x40*ch_num + 0x04 + DMA_base)
def DMA_TR_COUNT(ch_num):
    return (0x40*ch_num + 0x08 + DMA_base)
def DMA_CTRL(ch_num):
    return (0x40*ch_num + 0x0c + DMA_base)

# === Kill channels
# kill the channels by writing one to the bit number of thh channel
DMA_Ch_ABORT = 0x444 + DMA_base
DMA_BUSY = (1<<24) 
DMA_IRQ_QUIET = (1<<21) # bit 21 turn off interrupt

def DMA_TREQ(trigger_source):
    return (trigger_source & 0x3f)<<15
# When this channel completes, it will trigger the channel
# indicated by CHAIN_TO. Disable by setting CHAIN_TO =
# (this channel).
def DMA_CHAIN_TO (next_ch):
    return (next_ch & 0x0f)<<11 # bits 11:14 next channel #

DMA_WR_INC = (1<<5) # bits 5
DMA_RD_INC = (1<<4) # bits 4

data_8  = 0x00
data_16 = 0x01
data_32 = 0x02
def DMA_DATA_WIDTH(data_width):
    return (data_width & 0x03)<<2 # bits 2:3

# turn on the channel
DMA_EN = 1 # bits 0

DREQ_ADC = 36 # control DREQ ADC
DREQ_TIMER0 = 0x3b # data request source number Timer0
DREQ_PWM_WRAP0 = 24

# Buffers
pause_adc = array.array('i', [ADC_RROBIN(0x7)|ADC_AINSEL(0)|(0<<3)|ADC_EN])
run_adc   = array.array('i', [ADC_RROBIN(0x7)|ADC_AINSEL(0)|(1<<3)|ADC_EN])
adc_buff  = array.array('H', (0 for i in range(3)))

adc_buff1 = array.array('H', [0])
adc_buff2 = array.array('H', [0])
adc_buff3 = array.array('H', [0])

def get_data():
    return [adc_buff1[0], adc_buff2[0], adc_buff3[0]]

# ====================================
# DMA setup
print(bin(machine.mem32[ADC_CS]))


stp_adc_dma_chan = 3
adc_dma_chan = 2
adc_dma_chan3 = 6
adc_dma_chan2 = 5
adc_dma_chan1 = 4
sig_dma_chan = 1
rst_adc_dma_chan = 0

mask = 0
for i in range(7):
    mask |= 1 << i


# Pauses the ADC (after obtaining the 3 samples)
IOreg_write(DMA_RD_ADDR(stp_adc_dma_chan), addressof(pause_adc))
IOreg_write(DMA_WR_ADDR(stp_adc_dma_chan), ADC_CS)
IOreg_write(DMA_TR_COUNT(stp_adc_dma_chan), 1)
IOreg_write(DMA_CTRL(stp_adc_dma_chan), DMA_IRQ_QUIET |
                            DMA_DATA_WIDTH(data_32) |
                            DMA_CHAIN_TO(rst_adc_dma_chan) )

# adc 3 DMA channel 
# Gathers ADC samples and moves it into the buffer
IOreg_write(DMA_RD_ADDR(adc_dma_chan3), ADC_FIFO)
IOreg_write(DMA_WR_ADDR(adc_dma_chan3), addressof(adc_buff3))
IOreg_write(DMA_TR_COUNT(adc_dma_chan3), 1)
IOreg_write(DMA_CTRL(adc_dma_chan3), DMA_IRQ_QUIET |
                        DMA_TREQ(DREQ_ADC) |
                        DMA_DATA_WIDTH(data_32) |
                        DMA_CHAIN_TO(stp_adc_dma_chan) )
# adc2 DMA channel 
# Gathers ADC samples and moves it into the buffer
IOreg_write(DMA_RD_ADDR(adc_dma_chan2), ADC_FIFO)
IOreg_write(DMA_WR_ADDR(adc_dma_chan2), addressof(adc_buff2))
IOreg_write(DMA_TR_COUNT(adc_dma_chan2), 1)
IOreg_write(DMA_CTRL(adc_dma_chan2), DMA_IRQ_QUIET |
                        DMA_TREQ(DREQ_ADC) |
                        DMA_DATA_WIDTH(data_32) |
                        DMA_CHAIN_TO(adc_dma_chan3) )
# adc1 DMA channel 
# Gathers ADC samples and moves it into the buffer
IOreg_write(DMA_RD_ADDR(adc_dma_chan1), ADC_FIFO)
IOreg_write(DMA_WR_ADDR(adc_dma_chan1), addressof(adc_buff1))
IOreg_write(DMA_TR_COUNT(adc_dma_chan1), 1)
IOreg_write(DMA_CTRL(adc_dma_chan1), DMA_IRQ_QUIET |
                        DMA_TREQ(DREQ_ADC) |
                        DMA_DATA_WIDTH(data_32) |
                        DMA_CHAIN_TO(adc_dma_chan2) )






# Waits for the start of a PWM pulse to run ADC sampling
IOreg_write(DMA_RD_ADDR(sig_dma_chan), addressof(run_adc))
IOreg_write(DMA_WR_ADDR(sig_dma_chan), ADC_CS)
IOreg_write(DMA_TR_COUNT(sig_dma_chan), 1)
IOreg_write(DMA_CTRL(sig_dma_chan), DMA_IRQ_QUIET |
                            DMA_TREQ(DREQ_PWM_WRAP0) | 
                            DMA_DATA_WIDTH(data_32) |
                            DMA_CHAIN_TO(adc_dma_chan1) )


# Resets DMA2 to write at the beginning of the ADC buffer
IOreg_write(DMA_RD_ADDR(rst_adc_dma_chan), addressof(adc_buff))
IOreg_write(DMA_WR_ADDR(rst_adc_dma_chan), DMA_WR_ADDR(adc_dma_chan))
IOreg_write(DMA_TR_COUNT(rst_adc_dma_chan), 1)
IOreg_write(DMA_CTRL(rst_adc_dma_chan), DMA_IRQ_QUIET |
                            DMA_DATA_WIDTH(data_16) |
                            DMA_CHAIN_TO(sig_dma_chan) )




#machine.mem32[DMA_multi_chan_enable] |= mask # doesn't work

machine.mem32[DMA_CTRL(stp_adc_dma_chan)] |= DMA_EN
machine.mem32[DMA_CTRL(adc_dma_chan3)] |= DMA_EN
machine.mem32[DMA_CTRL(adc_dma_chan2)] |= DMA_EN
machine.mem32[DMA_CTRL(adc_dma_chan1)] |= DMA_EN
machine.mem32[DMA_CTRL(sig_dma_chan)] |= DMA_EN
machine.mem32[DMA_CTRL(rst_adc_dma_chan)] |= DMA_EN








#print(adc_buff)
print(get_data())
print(get_data())
print(get_data())
print(get_data())


try:
    while True:
        print(get_data())
except KeyboardInterrupt:
    print("Exiting")
      


"""
machine.mem32[DMA_Ch_ABORT] = 0xffff
machine.mem32[ADC_CS] |= (0&(0x1)) << 0
adc_fifo_drain(True)
IOreg_write(ADC_FIFO_CS, ADC_THRESH(1) |
                        ADC_DREQ_EN |
                        0)
IOreg_write(ADC_CS, (ADC_RROBIN(0x7) |
                        ADC_AINSEL(0) |
                        0))
print(bin(machine.mem32[ADC_CS]))
"""

