import rp2
import math
import time
import array
import machine

from uctypes import addressof

# Pins
hsc = 21 
hsb = 19 
hsa = 17 
lsc = 20 
lsb = 18 
lsa = 16 
pwm_ref = 14 # reference pwm
pwm_sam = 15 # sampling pwm



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

# === GPIO ===========================
GPIO_BASE         = 0xd0000000
GPIO_IN_ADDR      = 0x004 + GPIO_BASE
GPIO_OUT_ADDR     = 0x010 + GPIO_BASE
GPIO_OUT_SET_ADDR = 0x014 + GPIO_BASE
GPIO_OUT_CLR_ADDR = 0x018 + GPIO_BASE
GPIO_OE_ADDR      = 0x020 + GPIO_BASE
GPIO_OE_SET_ADDR  = 0x024 + GPIO_BASE
GPIO_OE_CLR_ADDR  = 0x028 + GPIO_BASE

def GPIO_CTRL_ADDR(chan_num):
    return chan_num*8 + 4 + IO_base

def GPIO_STATUS_ADDR(chan_num):
    return chan_num*8 + IO_base

def GPIO_PAD_CTRL_ADDR(pad_num):
    return (pad_num*4 + 4) + PADS_BASE

def GPIO2SliceNum(chan_num):
    return (chan_num >> 1) & 7



# gpio_init(lsa)
IOreg_write(GPIO_IN_ADDR, 1<<lsa)      # gpio_set_dir(lsa, GPIO.IN)
IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsa) # gpio_put(lsa, 0)
IOreg_write(GPIO_CTRL_ADDR(lsa), 5)    # gpio_set_function(lsa, 5)

# gpio_init(lsb)
IOreg_write(GPIO_IN_ADDR, 1<<lsb)      
IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsb) 
IOreg_write(GPIO_CTRL_ADDR(lsb), 5)

# gpio_init(lsc)
IOreg_write(GPIO_IN_ADDR, 1<<lsc)     
IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsc) 
IOreg_write(GPIO_CTRL_ADDR(lsc), 5)    


IOreg_write(GPIO_OE_ADDR, 1<<lsa)      # gpio_set_dir(lsa, GPIO.OUT)

# Needed to activate lsb without zeroing lsa
oe_mask = machine.mem32[GPIO_OE_ADDR] 

oe_mask |= 1<<lsb
IOreg_write(GPIO_OE_ADDR, oe_mask)

oe_mask |= 1<<lsc
IOreg_write(GPIO_OE_ADDR, oe_mask)



#IOreg_write(GPIO_OUT_SET_ADDR, 1<<lsa) # gpio_put(lsa, 1)
#IOreg_write(GPIO_OUT_SET_ADDR, 1<<lsc)

    
# === PWM ============================
PWM_base =  0x40050000
PWM_INTR_ADDR =  0xa4 + PWM_base
PWM_EN_ADDR   =  0xa0 + PWM_base

def GPIO2SliceNum(chan_num):
    return (chan_num >> 1) & 7

def GPIO2SliceLev(chan_num):
    return chan_num % 2

def PWM_CSR_ADDR(slice_num):
    return (0x14 * slice_num + PWM_base)

def PWM_divmode(mode):
    return (mode & 0x03)<<4

def PWM_DIV_ADDR(slice_num):
    return (0x14 * slice_num + PWM_base + 0x04)

# the actual PWM 16-bit counter
def PWM_CTR_ADDR(slice_num):
    return (0x14 * slice_num + PWM_base + 0x08)

# counter compare 31:16 B, 15:0 A
def PWM_CC_ADDR(slice_num):
    return (0x14 * slice_num + PWM_base + 0x0c)

# counter wrap 16 bit value
def PWM_TOP_ADDR(slice_num):
    return (0x14 * slice_num + PWM_base + 0x10)

# === Bits in (clock) divider
def PWM_DIV(i, f):
    return ((i & 0xff) << 4) | (f & 0xf)


# === Bits in pwm_cc
def PWM_CC(val, slice_num, level):
    if level:
        return (val & 0xffff) << 16
    else:
        return val

# ====================================
# PWM setup 
PWM_FN = 4
pwm_mask = 0
free_run = 0

# 20 kHz at full clock speed (125 MHz)
wrap = 6249 

# A few cc values for 20 kHz
cc_10us  = 1250
cc_9us   = 1125
cc_8us   = 1000
cc_7us   = 875
cc_6us   = 750
cc_5us   = 625
cc_4us   = 500
cc_3us   = 375
cc_2us   = 250
cc_1us   = 125
cc_0p5us = 62


# ====================================
# PWM initialization

# Disable all PWMs
IOreg_write(PWM_EN_ADDR, 0)

# Reset counters
IOreg_write(PWM_CTR_ADDR(GPIO2SliceNum(pwm_ref)), 0)
IOreg_write(PWM_CTR_ADDR(GPIO2SliceNum(pwm_sam)), 0)
IOreg_write(PWM_CTR_ADDR(GPIO2SliceNum(hsa)), 0)
IOreg_write(PWM_CTR_ADDR(GPIO2SliceNum(hsb)), 0)
IOreg_write(PWM_CTR_ADDR(GPIO2SliceNum(hsc)), 0)

# Define functions
IOreg_write(GPIO_CTRL_ADDR(pwm_ref), PWM_FN ) # pwm function slice 7A
IOreg_write(GPIO_CTRL_ADDR(pwm_sam), PWM_FN ) # pwm function slice 7B
IOreg_write(GPIO_CTRL_ADDR(hsa), PWM_FN ) # pwm function slice 0B
IOreg_write(GPIO_CTRL_ADDR(hsb), PWM_FN ) # pwm function slice 1B
IOreg_write(GPIO_CTRL_ADDR(hsc), PWM_FN ) # pwm function slice 2B






# Shift pwm_sam
shift = ((2**16)-1) - ((5*cc_1us))
IOreg_write(PWM_CTR_ADDR(GPIO2SliceNum(pwm_sam)), shift)


# pwm sampling channel
cc = cc_1us
sli = GPIO2SliceNum(pwm_sam)
lvl = GPIO2SliceLev(pwm_sam)
IOreg_write(PWM_DIV_ADDR(sli), PWM_DIV(1,0))
IOreg_write(PWM_TOP_ADDR(sli), wrap)
IOreg_write(PWM_CC_ADDR(sli), PWM_CC(cc, sli, lvl)) 
IOreg_write(PWM_CSR_ADDR(sli), PWM_divmode(free_run))
pwm_mask |= 1 << GPIO2SliceNum(pwm_sam)

# pwm hsa
cc = 0#1249
sli = GPIO2SliceNum(hsa)
lvl = GPIO2SliceLev(hsa)
IOreg_write(PWM_DIV_ADDR(sli), PWM_DIV(1,0))
IOreg_write(PWM_TOP_ADDR(sli), wrap)
IOreg_write(PWM_CC_ADDR(sli), PWM_CC(cc, sli, lvl)) 
IOreg_write(PWM_CSR_ADDR(sli), PWM_divmode(free_run))
pwm_mask |= 1 << sli

# pwm hsb
cc = 0#1249
sli = GPIO2SliceNum(hsb)
lvl = GPIO2SliceLev(hsb)
IOreg_write(PWM_DIV_ADDR(sli), PWM_DIV(1,0))
IOreg_write(PWM_TOP_ADDR(sli), wrap)
IOreg_write(PWM_CC_ADDR(sli), PWM_CC(cc, sli, lvl)) 
IOreg_write(PWM_CSR_ADDR(sli), PWM_divmode(free_run))
pwm_mask |= 1 << sli

# pwm hsc
cc = 0
sli = GPIO2SliceNum(hsc)
lvl = GPIO2SliceLev(hsc)
IOreg_write(PWM_DIV_ADDR(sli), PWM_DIV(1,0))
IOreg_write(PWM_TOP_ADDR(sli), wrap)
IOreg_write(PWM_CC_ADDR(sli), PWM_CC(cc, sli, lvl)) 
IOreg_write(PWM_CSR_ADDR(sli), PWM_divmode(free_run))
pwm_mask |= 1 << sli


IOreg_write(PWM_EN_ADDR, pwm_mask) # enable PWM simultaneously

# ====================================
# PIO (P. 367)
PIO0_BASE_ADDR = 0x50200000
PIO0_TXF0_ADDR = 0x010 + PIO0_BASE_ADDR
PIO0_TXF1_ADDR = 0x014 + PIO0_BASE_ADDR
PIO0_TXF2_ADDR = 0x018 + PIO0_BASE_ADDR
PIO0_RXF0_ADDR = 0x020 + PIO0_BASE_ADDR
PIO0_RXF1_ADDR = 0x024 + PIO0_BASE_ADDR
PIO0_RXF2_ADDR = 0x028 + PIO0_BASE_ADDR


# === ADC ============================
ADC_BASE         = 0x4004c000
ADC_CS_ADDR      = 0x00 + ADC_BASE
ADC_RESULT_ADDR  = 0x04 + ADC_BASE
ADC_FIFO_CS_ADDR = 0x08 + ADC_BASE
ADC_FIFO_ADDR    = 0x0c + ADC_BASE
ADC_DIV_ADDR     = 0x10 + ADC_BASE


def ADC_RROBIN(channels):
    return (channels & 0x1f)<<16

def ADC_AINSEL(channel):
    return (channel & 0x07)<<12

def ADC_READY(ready):
    return (ADC_CS & (1<<8))

ADC_START_MANY = (1<<3)
ADC_START_ONCE = (1<<2)
ADC_TS_EN = (1<<1) # Temperature sensor
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
    while ((machine.mem32[ADC_FIFO_CS_ADDR]>>16)&(0xf)):
        x = machine.mem32[ADC_FIFO_ADDR]
        if(p):
            print(x)
    


#==== ADC Setup ============================

# Output disable, input disable, PUE and PDE disable
IOreg_write(GPIO_PAD_CTRL_ADDR(26), 0b10000000)
#IOreg_write(GPIO_PAD_CTRL_ADDR(27), 0b10000000)
#IOreg_write(GPIO_PAD_CTRL_ADDR(28), 0b10000000)

IOreg_write(ADC_DIV_ADDR, ADC_Div_int(96) | ADC_Div_frac(0))

IOreg_write(ADC_CS_ADDR, (ADC_RROBIN(0x7) |
                        ADC_AINSEL(0) |
                        ADC_EN))


adc_fifo_drain(0)

IOreg_write(ADC_FIFO_CS_ADDR, (ADC_THRESH(1) |
                        ADC_DREQ_EN |
                        ADC_FIFO_EN) )



#==== DMA ============================
# register adddresses for DMA
# datasheet page 102
DMA_base              = 0x50000000
DMA_multi_chan_enable = 0x430 + DMA_base
DMA_TIMER0            = 0x420 + DMA_base
DMA_Ch_ABORT_ADDR     = 0x444 + DMA_base
# ===================================
# === DMA channel control registers
# valid channels: 0 to 11
def DMA_RD_ADDR(ch_num):
    return (0x40*ch_num + DMA_base)

def DMA_WR_ADDR(ch_num):
    return (0x40*ch_num + 0x04 + DMA_base)

def DMA_TR_COUNT_ADDR(ch_num):
    return (0x40*ch_num + 0x08 + DMA_base)

def DMA_CTRL_ADDR(ch_num):
    return (0x40*ch_num + 0x0c + DMA_base)



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

DMA_EN = 1


DREQ_PIO0_TX0 = 0
DREQ_PIO0_RX0 = 4
DREQ_ADC = 36
DREQ_PWM_WRAP0 = 24
DREQ_PWM_WRAP1 = 25
DREQ_PWM_WRAP2 = 26
DREQ_PWM_WRAP7 = 31
DREQ_UNPACED = 0x3f



# ====================================
# DMA setup

pulse_array = array.array('i', [PWM_CC((1 * cc_10us), 0, 1)]*150)
discharge_array = array.array('i', [0,
                                    0,
                                    0,
                                    0,
                                    0])
nPulses = len(pulse_array)
nDischarges = len(discharge_array)
adc_buf_array = array.array('i', [0]*(nPulses + nDischarges))





adc_buf_dma_chan = 8
ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_ADC) |
        DMA_WR_INC |
        DMA_DATA_WIDTH(data_32))

IOreg_write(DMA_RD_ADDR(adc_buf_dma_chan), ADC_FIFO_ADDR)
IOreg_write(DMA_WR_ADDR(adc_buf_dma_chan), addressof(adc_buf_array))
IOreg_write(DMA_TR_COUNT_ADDR(adc_buf_dma_chan), len(adc_buf_array))
IOreg_write(DMA_CTRL_ADDR(adc_buf_dma_chan),  ctrl)
machine.mem32[DMA_CTRL_ADDR(adc_buf_dma_chan)] |= DMA_EN






run_adc_dma_chan = 7
sample_lsa = array.array('i', [ADC_RROBIN(0)|ADC_AINSEL(0)|(1<<2)|ADC_EN])
sample_lsb = array.array('i', [ADC_RROBIN(0)|ADC_AINSEL(1)|(1<<2)|ADC_EN])
sample_lsc = array.array('i', [ADC_RROBIN(0)|ADC_AINSEL(2)|(1<<2)|ADC_EN])
ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_PWM_WRAP7) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(run_adc_dma_chan), addressof(sample_lsa))
IOreg_write(DMA_WR_ADDR(run_adc_dma_chan), ADC_CS_ADDR)
IOreg_write(DMA_TR_COUNT_ADDR(run_adc_dma_chan), len(adc_buf_array))
IOreg_write(DMA_CTRL_ADDR(run_adc_dma_chan),  ctrl)
machine.mem32[DMA_CTRL_ADDR(run_adc_dma_chan)] |= DMA_EN



sync_array = array.array('i', [0, 0])
sync_adc_dma_chan = 6
ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_PWM_WRAP0) |
        DMA_CHAIN_TO(run_adc_dma_chan) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(sync_adc_dma_chan), addressof(sync_array))
IOreg_write(DMA_WR_ADDR(sync_adc_dma_chan), addressof(sync_array))
IOreg_write(DMA_TR_COUNT_ADDR(sync_adc_dma_chan), len(sync_array))
IOreg_write(DMA_CTRL_ADDR(sync_adc_dma_chan),  ctrl)
machine.mem32[DMA_CTRL_ADDR(sync_adc_dma_chan)] |= DMA_EN








pulse_finished_array = array.array('i', [5])
pulse_finished_dma_chan = 5

ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_UNPACED) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(pulse_finished_dma_chan), addressof(pulse_finished_array))
IOreg_write(DMA_WR_ADDR(pulse_finished_dma_chan), PIO0_TXF2_ADDR)
IOreg_write(DMA_TR_COUNT_ADDR(pulse_finished_dma_chan), 1)
IOreg_write(DMA_CTRL_ADDR(pulse_finished_dma_chan), ctrl)
machine.mem32[DMA_CTRL_ADDR(pulse_finished_dma_chan)] |= DMA_EN






discharge_dma_chan = 4

ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_PWM_WRAP0) |
        DMA_RD_INC |
        DMA_CHAIN_TO(pulse_finished_dma_chan) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(discharge_dma_chan), addressof(discharge_array))
IOreg_write(DMA_TR_COUNT_ADDR(discharge_dma_chan), nDischarges)
IOreg_write(DMA_CTRL_ADDR(discharge_dma_chan), ctrl)
machine.mem32[DMA_CTRL_ADDR(discharge_dma_chan)] |= DMA_EN


discharge_sig_array = array.array('i', [3])
discharge_sig_dma_chan = 3

ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_UNPACED) |
        DMA_CHAIN_TO(discharge_dma_chan) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(discharge_sig_dma_chan), addressof(discharge_sig_array))
IOreg_write(DMA_WR_ADDR(discharge_sig_dma_chan), PIO0_TXF1_ADDR)
IOreg_write(DMA_TR_COUNT_ADDR(discharge_sig_dma_chan), 1)
IOreg_write(DMA_CTRL_ADDR(discharge_sig_dma_chan), ctrl)
machine.mem32[DMA_CTRL_ADDR(discharge_sig_dma_chan)] |= DMA_EN

pulse_dma_chan = 2

ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_PWM_WRAP0) |
        DMA_RD_INC |
        DMA_CHAIN_TO(discharge_sig_dma_chan) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(pulse_dma_chan), addressof(pulse_array))
IOreg_write(DMA_TR_COUNT_ADDR(pulse_dma_chan), nPulses)
IOreg_write(DMA_CTRL_ADDR(pulse_dma_chan), ctrl)
machine.mem32[DMA_CTRL_ADDR(pulse_dma_chan)] |= DMA_EN






sig_adc_dma_chan = 1
sig_adc_array = array.array('i', [1])
ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_PIO0_RX0) |
        DMA_CHAIN_TO(sync_adc_dma_chan) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(sig_adc_dma_chan), addressof(sig_adc_array))
IOreg_write(DMA_WR_ADDR(sig_adc_dma_chan), addressof(sig_adc_array))
IOreg_write(DMA_TR_COUNT_ADDR(sig_adc_dma_chan), 1)
IOreg_write(DMA_CTRL_ADDR(sig_adc_dma_chan),  ctrl)
machine.mem32[DMA_CTRL_ADDR(sig_adc_dma_chan)] |= DMA_EN


sig_array = array.array('i', [0])
sig_pulse_dma_chan = 0

ctrl = (DMA_IRQ_QUIET |
        DMA_TREQ(DREQ_PIO0_RX0) |
        DMA_CHAIN_TO(pulse_dma_chan) |
        DMA_DATA_WIDTH(data_32) )

IOreg_write(DMA_RD_ADDR(sig_pulse_dma_chan), addressof(sig_array))
IOreg_write(DMA_WR_ADDR(sig_pulse_dma_chan), addressof(sig_array))
IOreg_write(DMA_TR_COUNT_ADDR(sig_pulse_dma_chan), 1)
IOreg_write(DMA_CTRL_ADDR(sig_pulse_dma_chan), ctrl)
machine.mem32[DMA_CTRL_ADDR(sig_pulse_dma_chan)] |= DMA_EN
    



mask = 0
mask |= 1 << sig_pulse_dma_chan
mask |= 1 << sig_adc_dma_chan
mask |= 1 << adc_buf_dma_chan








# ====================================
# PIO setup
@rp2.asm_pio()
def PIORXSignal():
    label("wait4data")
    pull()
    jmp(not_osre, "signal")
    jmp("wait4data")
    label("signal")
    mov(isr, osr)
    push()
    jmp("wait4data")


rp2.PIO(0).remove_program()
activate_sm = rp2.StateMachine(0, prog=PIORXSignal)
activate_sm.active(1)
discharge_sig_sm = rp2.StateMachine(1, prog=PIORXSignal)
discharge_sig_sm.active(1)
pulse_finished_sm = rp2.StateMachine(2, prog=PIORXSignal)
pulse_finished_sm.active(1)



# ====================================
# Next phase function
def NextPhase(next):
    if ((next >> 2) & 1):
        return 1
    elif ((next >> 1) & 1):
        return 0
    else:
        return -1




# ====================================
# Pulses function
def Pulses(hsx):
    # Reset the read address
    IOreg_write(DMA_RD_ADDR(pulse_dma_chan), addressof(pulse_array))
    IOreg_write(DMA_RD_ADDR(discharge_dma_chan), addressof(discharge_array))
    # Update the pulse pin
    IOreg_write(DMA_WR_ADDR(pulse_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsx)))
    IOreg_write(DMA_WR_ADDR(discharge_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsx)))
    return

# ====================================
# Sample function
def Sample(lsx):
    # Reset the adc buffer write address
    IOreg_write(DMA_WR_ADDR(adc_buf_dma_chan), addressof(adc_buf_array))
    
    if lsx == lsa:
        sample_lsx = sample_lsa
    elif lsx == lsb:
        sample_lsx = sample_lsb
    else:
        sample_lsx = sample_lsc
        
    # Update the ADC channel pin
    IOreg_write(DMA_RD_ADDR(run_adc_dma_chan), addressof(sample_lsx))
    return
    
# ====================================
# Pulse sampling functions
def RunPulses(hsx, lsx):
    # Reset the adc buffer write address and prepare the sampling DMA
    Sample(lsx)
    # Prepare the pulse DMA
    Pulses(hsx)
    # Enable the required DMA channels
    IOreg_write(DMA_multi_chan_enable, mask)
    # Set lsx high
    IOreg_write(GPIO_OUT_SET_ADDR, 1<<lsx)
    # Activate hsx pulse
    activate_sm.put(42069)
    a = activate_sm.get()
    # Wait for discharge state
    current_phase = discharge_sig_sm.get() 
    next_phase = (current_phase + NextPhase(0b100)) % 6
    discharge_sig_array[0] = next_phase
    c = pulse_finished_sm.get()
    # Set lsx low
    IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsx)
    return current_phase, adc_buf_array

# ====================================
# Single Pulse function
def Pulse(hsx, lsx):
    # Reset the adc buffer write address and prepare the sampling DMA
    Sample(lsx)
    # Reset the read address
    IOreg_write(DMA_RD_ADDR(pulse_dma_chan), addressof(pulse_array))
    IOreg_write(DMA_RD_ADDR(discharge_dma_chan), addressof(discharge_array))
    # Update the pulse pin
    IOreg_write(DMA_WR_ADDR(pulse_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsx)))
    IOreg_write(DMA_WR_ADDR(discharge_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsx)))
    # Single pulse means changing the appropriate transfer counts
    IOreg_write(DMA_TR_COUNT_ADDR(pulse_dma_chan), 1)
    IOreg_write(DMA_TR_COUNT_ADDR(run_adc_dma_chan), 1)
    
    # Enable the required DMA channels
    IOreg_write(DMA_multi_chan_enable, mask)
    # Set lsx high
    IOreg_write(GPIO_OUT_SET_ADDR, 1<<lsx)
    # Activate hsx pulse
    activate_sm.put(42069)
    a = activate_sm.get()
    # Wait for discharge state
    current_phase = discharge_sig_sm.get()
    c = pulse_finished_sm.get()
    # Set lsx low
    IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsx)
    
    # Restoring the transfer counts
    IOreg_write(DMA_TR_COUNT_ADDR(pulse_dma_chan), nPulses)
    IOreg_write(DMA_TR_COUNT_ADDR(run_adc_dma_chan), len(adc_buf_array))
    
    return adc_buf_array[0]

# ====================================
# Position detection function
def PosDet():
    r = [0, 0, 0]
    
    # Single pulse means changing the appropriate transfer counts
    IOreg_write(DMA_TR_COUNT_ADDR(pulse_dma_chan), 1)
    IOreg_write(DMA_TR_COUNT_ADDR(run_adc_dma_chan), 1)
    
    # Starting with lsc, run pulse on hsb
    # Reset the adc buffer write address and prepare the sampling DMA
    Sample(lsc)
    
    # Reset the pulse read address
    IOreg_write(DMA_RD_ADDR(pulse_dma_chan), addressof(pulse_array))
    IOreg_write(DMA_RD_ADDR(discharge_dma_chan), addressof(discharge_array))
    # Running pulse on hsb
    # Update the pulse pin
    IOreg_write(DMA_WR_ADDR(pulse_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsb)))
    IOreg_write(DMA_WR_ADDR(discharge_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsb)))
    
    # Enable the required DMA channels
    IOreg_write(DMA_multi_chan_enable, mask)
    # Set lsc high
    IOreg_write(GPIO_OUT_SET_ADDR, 1<<lsc)
    # Activate hsb pulse
    activate_sm.put(42069)
    a = activate_sm.get()
    # Wait for discharge state
    current_phase = discharge_sig_sm.get()
    # Update position 0
    r[2] = adc_buf_array[0]
    # Wait until discharge state is finished
    c = pulse_finished_sm.get()
    
    # Set lsc low
    IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsc)
    
    
    
    
    # Then to lsa, run pulse on hsc and hsb
    # Reset the adc buffer write address and prepare the sampling DMA
    Sample(lsa)
    
    # Reset the pulse read address
    IOreg_write(DMA_RD_ADDR(pulse_dma_chan), addressof(pulse_array))
    IOreg_write(DMA_RD_ADDR(discharge_dma_chan), addressof(discharge_array))
    # Running pulse on hsa
    # Update the pulse pin
    IOreg_write(DMA_WR_ADDR(pulse_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsc)))
    IOreg_write(DMA_WR_ADDR(discharge_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsc)))
    
    # Enable the required DMA channels
    IOreg_write(DMA_multi_chan_enable, mask)
    # Set lsc high
    IOreg_write(GPIO_OUT_SET_ADDR, 1<<lsa)
    # Activate hsc pulse
    activate_sm.put(42069)
    a = activate_sm.get()
    # Wait for discharge state
    current_phase = discharge_sig_sm.get()
    # Update position 0
    r[0] = adc_buf_array[0]
    # Wait until discharge state is finished
    c = pulse_finished_sm.get()
    
    # Reset the adc buffer write address
    IOreg_write(DMA_WR_ADDR(adc_buf_dma_chan), addressof(adc_buf_array))
    # Reset the pulse read address
    IOreg_write(DMA_RD_ADDR(pulse_dma_chan), addressof(pulse_array))
    IOreg_write(DMA_RD_ADDR(discharge_dma_chan), addressof(discharge_array))
    # Running pulse on hsb
    # Update the pulse pin
    IOreg_write(DMA_WR_ADDR(pulse_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsb)))
    IOreg_write(DMA_WR_ADDR(discharge_dma_chan), PWM_CC_ADDR(GPIO2SliceNum(hsb)))
    # Enable the required DMA channels
    IOreg_write(DMA_multi_chan_enable, mask)
    # Activate hsb pulse
    activate_sm.put(42069)
    a = activate_sm.get()
    # Wait for discharge state
    current_phase = discharge_sig_sm.get()
    # Update position 1
    r[1] = adc_buf_array[0]
    # Wait until discharge state is finished
    c = pulse_finished_sm.get()
    
    # Set lsa low
    IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsa)
    
    # Restoring the transfer counts
    IOreg_write(DMA_TR_COUNT_ADDR(pulse_dma_chan), nPulses)
    IOreg_write(DMA_TR_COUNT_ADDR(run_adc_dma_chan), len(adc_buf_array))
    
    return r
    






adc_fifo_drain(0)

phases = [ (hsc, lsa),
           (hsb, lsa),
           (hsb, lsc),
           (hsa, lsc),
           (hsa, lsb),
           (hsc, lsb) ]

#def MotorPhase0():
#def MotorPhase1():
#def MotorPhase2():
#def MotorPhase3():
#def MotorPhase4():
#def MotorPhase5():

# Initial Position detection goes here
phase = 5
discharge_sig_array[0] = phase
hsx, lsx = phases[phase]

p = [[0,3], [1, 4], [2, 5]]

try:
    while True:
        a = PosDet()
        pos = a.index(max(a))
        nxt = a.index(min(a))
        
        print(p[pos], p[nxt], a)
        
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
    IOreg_write(GPIO_OUT_CLR_ADDR, 1<<lsc)
    activate_sm.active(0)
    discharge_sig_sm.active(0)
    pulse_finished_sm.active(0)
    rp2.PIO(0).remove_program()
    machine.mem32[DMA_Ch_ABORT_ADDR] = 0xffff
    adc_fifo_drain(1)

