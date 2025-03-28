import array
import time
import machine
import math
from uctypes import addressof

# Pins
hsc = 20
hsb = 18
hsa = 16
lsc = 21
lsb = 19
lsa = 17
pwm_ref = 14

LSA = machine.Pin(lsa, machine.Pin.OUT)
LSA.high()


# ====================================
# === Register write functions =======

def IOreg_write(reg_addr, data):
    machine.mem32[reg_addr] = data
    
def IOreg_set(reg_addr, data):
    machine.mem32[reg_addr + 0x2000] = data

def IOreg_clr(reg_addr, data):
    machine.mem32[reg_addr + 0x3000] = data
   
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
PWM_base =  0x40050000
PWM_INTR =  0xa4 + PWM_base
PWM_EN   =  0xa0 + PWM_base

def PWM_CSR(slice_num):
    return (0x14 * slice_num + PWM_base)

def PWM_divmode(mode):
    return (mode & 0x03)<<4

PWM_B_inv = 1<<3
PWM_A_inv = 1<<2
PWM_ph_correct = 1<<1

def PWM_DIV(slice_num):
    return (0x14 * slice_num + PWM_base + 0x04)

# the actual PWM 16-bit counter
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
pwm_mask = 0
free_run = 0

# Disable all PWMs
IOreg_write(PWM_EN, 0)



# Reset counters
IOreg_write(PWM_CTR(GPIO2SliceNum(pwm_ref)), 0)
IOreg_write(PWM_CTR(GPIO2SliceNum(hsa)), 0)
IOreg_write(PWM_CTR(GPIO2SliceNum(hsb)), 0)
IOreg_write(PWM_CTR(GPIO2SliceNum(hsc)), 0)

IOreg_write(GPIO_CTRL(pwm_ref), PWM_FN ) # pwm function slice 0A
IOreg_write(GPIO_CTRL(hsa), PWM_FN ) # pwm function slice 1A
IOreg_write(GPIO_CTRL(hsb), PWM_FN ) # pwm function slice 2A
IOreg_write(GPIO_CTRL(hsc), PWM_FN ) # pwm function slice 3A

# Shift PWM0
shift = ((2**16)-1) - 625 #5us
IOreg_write(PWM_CTR(GPIO2SliceNum(pwm_ref)), shift)

# Shift PWM4
shift = ((2**16)-1) - 250 #2us
#IOreg_write(PWM_CTR(GPIO2SliceNum(hsb)), shift)

# Shift PWM6
shift = ((2**16)-1) - 500 #4us
#IOreg_write(PWM_CTR(GPIO2SliceNum(hsc)), shift)




# pwm reference channel
wrapVal = 6249
cc = 750 
IOreg_write(PWM_DIV(GPIO2SliceNum(pwm_ref)), 1<<4)
IOreg_write(PWM_TOP(GPIO2SliceNum(pwm_ref)), wrapVal)
IOreg_write(PWM_CC(GPIO2SliceNum(pwm_ref)), cc) 
IOreg_write(PWM_CSR(GPIO2SliceNum(pwm_ref)), PWM_divmode(free_run))
pwm_mask |= 1 << GPIO2SliceNum(pwm_ref)

# pwm hsa
wrapVal = 6249
cc = 0#1249 
IOreg_write(PWM_DIV(GPIO2SliceNum(hsa)), 1<<4)
IOreg_write(PWM_TOP(GPIO2SliceNum(hsa)), wrapVal)
IOreg_write(PWM_CC(GPIO2SliceNum(hsa)), cc) 
IOreg_write(PWM_CSR(GPIO2SliceNum(hsa)), PWM_divmode(free_run))
pwm_mask |= 1 << GPIO2SliceNum(hsa)

# pwm hsb
wrapVal = 6249
cc = 0 #1249 
IOreg_write(PWM_DIV(GPIO2SliceNum(hsb)), 1<<4)
IOreg_write(PWM_TOP(GPIO2SliceNum(hsb)), wrapVal) 
IOreg_write(PWM_CC(GPIO2SliceNum(hsb)), cc) 
IOreg_write(PWM_CSR(GPIO2SliceNum(hsb)), PWM_divmode(free_run))
pwm_mask |= 1 << GPIO2SliceNum(hsb)

# pwm hsc
wrapVal = 6249
cc = 0#1249 
IOreg_write(PWM_DIV(GPIO2SliceNum(hsc)), 1<<4)
IOreg_write(PWM_TOP(GPIO2SliceNum(hsc)), wrapVal)
IOreg_write(PWM_CC(GPIO2SliceNum(hsc)), cc) 
IOreg_write(PWM_CSR(GPIO2SliceNum(hsc)), PWM_divmode(free_run))
pwm_mask |= 1 << GPIO2SliceNum(hsc)


IOreg_write(PWM_EN, pwm_mask) # enable PWM simultaneously

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
ADC_SHIFT = (1<<1)
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

IOreg_write(ADC_DIV, ADC_Div_int(96) | ADC_Div_frac(0))

print(bin(machine.mem32[ADC_CS]))
IOreg_write(ADC_CS, (ADC_RROBIN(0x7) |
                        ADC_AINSEL(0) |
                        ADC_EN))
print(bin(machine.mem32[ADC_CS]))


adc_fifo_drain(1)

IOreg_write(ADC_FIFO_CS, ADC_THRESH(1) |
                        ADC_DREQ_EN |
                        ADC_FIFO_EN )



#==== DMA ============================
# register adddresses for DMA
# datasheet page 102
DMA_base = 0x50000000
DMA_multi_chan_enable = DMA_base + 0x430
DMA_TIMER0 = 0x420 + DMA_base
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


DMA_Ch_ABORT = 0x444 + DMA_base
DMA_BUSY = (1<<24) 
DMA_IRQ_QUIET = (1<<21) 

def DMA_TREQ(trigger_source):
    return (trigger_source & 0x3f)<<15

def DMA_CHAIN_TO (next_ch):
    return (next_ch & 0x0f)<<11

DMA_WR_INC = (1<<5) 
DMA_RD_INC = (1<<4) 


data_8  = 0x00
data_16 = 0x01
data_32 = 0x02
def DMA_DATA_WIDTH(data_width):
    return (data_width & 0x03)<<2

DMA_HIGH_PRI = (1<<1) 
DMA_EN = 1

DREQ_ADC = 36
DREQ_PWM_WRAP0 = 24



# ====================================
# DMA setup
stp_adc_dma_chan = 3
adc_dma_chan = 2
adc_dma_chan3 = 6
adc_dma_chan2 = 5
adc_dma_chan1 = 4
sig_dma_chan = 1
rst_adc_dma_chan = 0
tmr_dma_chan = 7

dma_mask = 0
for i in range(7):
    dma_mask |= 1 << i
    
# Buffers
pause_adc = array.array('i', [ADC_RROBIN(0x7)|ADC_AINSEL(0)|(0<<3)|ADC_EN])
run_adc   = array.array('i', [ADC_RROBIN(0x7)|ADC_AINSEL(0)|(1<<3)|ADC_EN])
pause_sig_dma = array.array('i', [DMA_IRQ_QUIET |
                            DMA_TREQ(DREQ_PWM_WRAP0) | 
                            DMA_DATA_WIDTH(data_32) |
                            DMA_CHAIN_TO(adc_dma_chan1)])
a = machine.mem32[PWM_EN]
a &= ~1
pause_pwm = array.array('i', [a])

adc_buff1 = array.array('H', [0])
adc_buff2 = array.array('H', [0])
adc_buff3 = array.array('H', [0])

def get_data():
    return [adc_buff1[0], adc_buff2[0], adc_buff3[0]]


# Pauses the ADC (after obtaining the 3 samples)
IOreg_write(DMA_RD_ADDR(stp_adc_dma_chan), addressof(pause_adc))
IOreg_write(DMA_WR_ADDR(stp_adc_dma_chan), ADC_CS)
IOreg_write(DMA_TR_COUNT(stp_adc_dma_chan), 1)
IOreg_write(DMA_CTRL(stp_adc_dma_chan), DMA_IRQ_QUIET |
                            DMA_DATA_WIDTH(data_32)  |
                            DMA_CHAIN_TO(sig_dma_chan) )


# adc 3 DMA channel 
# Gathers ADC samples and moves it into the buffer
IOreg_write(DMA_RD_ADDR(adc_dma_chan3), ADC_FIFO)
IOreg_write(DMA_WR_ADDR(adc_dma_chan3), addressof(adc_buff3))
IOreg_write(DMA_TR_COUNT(adc_dma_chan3), 1)
IOreg_write(DMA_CTRL(adc_dma_chan3), DMA_IRQ_QUIET |
                        DMA_TREQ(DREQ_ADC) |
                        DMA_DATA_WIDTH(data_16) |
                        DMA_CHAIN_TO(stp_adc_dma_chan) )


# adc2 DMA channel 
# Gathers ADC samples and moves it into the buffer
IOreg_write(DMA_RD_ADDR(adc_dma_chan2), ADC_FIFO)
IOreg_write(DMA_WR_ADDR(adc_dma_chan2), addressof(adc_buff2))
IOreg_write(DMA_TR_COUNT(adc_dma_chan2), 1)
IOreg_write(DMA_CTRL(adc_dma_chan2), DMA_IRQ_QUIET |
                        DMA_TREQ(DREQ_ADC) |
                        DMA_DATA_WIDTH(data_16) |
                        DMA_CHAIN_TO(adc_dma_chan3) )


# adc1 DMA channel 
# Gathers ADC samples and moves it into the buffer
IOreg_write(DMA_RD_ADDR(adc_dma_chan1), ADC_FIFO)
IOreg_write(DMA_WR_ADDR(adc_dma_chan1), addressof(adc_buff1))
IOreg_write(DMA_TR_COUNT(adc_dma_chan1), 1)
IOreg_write(DMA_CTRL(adc_dma_chan1), DMA_IRQ_QUIET |
                        DMA_TREQ(DREQ_ADC) |
                        DMA_DATA_WIDTH(data_16) |
                        DMA_CHAIN_TO(adc_dma_chan2) )



# Waits for the start of a PWM pulse to run ADC sampling
IOreg_write(DMA_RD_ADDR(sig_dma_chan), addressof(run_adc))
IOreg_write(DMA_WR_ADDR(sig_dma_chan), ADC_CS)
IOreg_write(DMA_TR_COUNT(sig_dma_chan), 1)
IOreg_write(DMA_CTRL(sig_dma_chan), DMA_IRQ_QUIET |
                            DMA_TREQ(DREQ_PWM_WRAP0) | 
                            DMA_DATA_WIDTH(data_32) |
                            DMA_CHAIN_TO(adc_dma_chan1) )



#machine.mem32[DMA_multi_chan_enable] |= dma_mask # doesn't work
machine.mem32[DMA_CTRL(stp_adc_dma_chan)] |= DMA_EN
machine.mem32[DMA_CTRL(adc_dma_chan3)] |= DMA_EN
machine.mem32[DMA_CTRL(adc_dma_chan2)] |= DMA_EN
machine.mem32[DMA_CTRL(adc_dma_chan1)] |= DMA_EN
machine.mem32[DMA_CTRL(sig_dma_chan)] |= DMA_EN


try:
    while True:
        SOA = get_data()[0]
        if SOA > 3500:
            IOreg_write(PWM_CC(GPIO2SliceNum(hsc)), 0)
        if SOA < 2000:
            IOreg_write(PWM_CC(GPIO2SliceNum(hsc)), 1249)
except KeyboardInterrupt:
    print("Exiting")
    machine.mem32[DMA_Ch_ABORT] = 0xffff
    
    IOreg_write(ADC_FIFO_CS, ADC_THRESH(1) |
                        ADC_DREQ_EN |
                        0 )
    IOreg_write(ADC_CS, (ADC_RROBIN(0x7) |
                        ADC_AINSEL(0) |
                        0))
    adc_fifo_drain(True)
   
