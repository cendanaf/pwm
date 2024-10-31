#from machine import Pin, PWM, ADC, mem32
from machine import Pin, ADC, mem32, PWM
import uctypes
import array
import math
import utime
import sys
import rp2

voltage_resolution = 16
nVoltages = (2**voltage_resolution) - 1

pwm_pins = [16,18,20]

adc_pins = [26, 27, 28]
pwm_reference_pin = 7
pio_reference_pin = 8
dma_channel = 0

#############################
######  PAD Registers  ######
#############################
PAD_BASE            = 0x4001c000
PAD_CHANNEL_WIDTH   = 0x04
PAD_PIN_COUNT       = 30

#############################
######  GPIO Registers  #####
#############################
GPIO_BASE       = 0x40014000
GPIO_CHANNEL_WIDTH = 0x08
GPIO_PIN_COUNT  = 30

GPIO_FUNC_SPI = 1
GPIO_FUNC_UART = 2
GPIO_FUNC_I2C = 3
GPIO_FUNC_PWM = 4
GPIO_FUNC_SIO = 5
GPIO_FUNC_PIO0 = 6
GPIO_FUNC_NULL = 0x1f

def gpio_set_function(gpio, func):
    pad_offset = (gpio + 1) * PAD_CHANNEL_WIDTH
    gpio_offset = gpio * GPIO_CHANNEL_WIDTH
    pad = PAD_BASE | (0x00 + pad_offset)
    gpio_ctrl = GPIO_BASE | (0x04 + gpio_offset)
    # Enable output disable PAD_OD(bit 7)
    mem32[pad] |= (0 & 0x1) << 7
    # Do not enable input enable PAD_IE(bit 6)
    mem32[pad] |= (1 & 0x1) << 6
    mem32[gpio_ctrl] |= (func & 0x1f) << 0
    
    

###########################
######  PWM Registers #####
###########################
PWM_BASE          = 0x40050000
PWM_CHANNEL_WIDTH = 0x14
PWM_EN  = PWM_BASE + 0xa0
"""
class PWM:
    def __init__(self, gpio, clock=125_000_000):
        self.gpio = gpio
        self.clock = clock
        self.slice_num = (gpio >> 1) & 7
        self.slice_offset = self.slice_num * PWM_CHANNEL_WIDTH
        self.CSR = PWM_BASE | (0x00 + self.slice_offset) # Control and status
        self.DIV = PWM_BASE | (0x04 + self.slice_offset) # Clock divider
        self.CTR = PWM_BASE | (0x08 + self.slice_offset) # PWM counter
        self.CC  = PWM_BASE | (0x0c + self.slice_offset) # Counter compare
        self.TOP = PWM_BASE | (0x10 + self.slice_offset) # Counter wrap
        
        mem32[self.CSR] = 0
        mem32[self.DIV] = 0
        mem32[self.CTR] = 0
        mem32[self.CC]  = 0
        mem32[self.TOP] = 0
        
        self.set_clkdiv_int_frac(1, 0)
        self.set_wrap(0xffff)
        
        
    def set_clkdiv_int_frac(self, i, f):
        mem32[self.DIV] |= (i & 0xff) << 4 #  INT (Bits 4:11)
        mem32[self.DIV] |= (f & 0xf)  << 0 #  FRAC (Bits 0:3)
        
    def set_clkdiv(self, i):
        self.set_clkdiv_int_frac(i, 0)
        
    def set_wrap(self, wrap):
        mem32[self.TOP] |= (wrap & 0xffff) << 0 # TOP (Bits 0:15)
        
    def set_chan_level(self, chan, level):
        if chan:
            mem32[self.CC] |= (level & 0xffff) << 16 # Channel B (Bits 31:16)
        else:
            mem32[self.CC] |= (level & 0xffff) << 0  # Channel A (Bits 15:0)
            
    def gpio_to_channel(self, gpio):
        return (gpio & 1)
            
    def set_enabled(self, en):
        mem32[self.CSR] |= (en & 0x1) << 0 # CSR_EN (Bit 0)
"""
#############################
######  ADC Registers  ######
#############################
ADC_BASE      = 0x4004c000
ADC_CS        = ADC_BASE + 0x00
ADC_RESULT    = ADC_BASE + 0x04
ADC_FCS       = ADC_BASE + 0x08
ADC_IRQ_FIFO  = ADC_BASE + 0x0c
ADC_DIV       = ADC_BASE + 0x10

def adc_set_clkdiv(div):
    mem32[ADC_DIV] = div
    
def adc_gpio_init(gpio):
    pad_offset = (gpio + 1) * PAD_CHANNEL_WIDTH
    gpio_offset = gpio * GPIO_CHANNEL_WIDTH
    pad = PAD_BASE | (0x00 + pad_offset)
    gpio_ctrl = GPIO_BASE | (0x04 + gpio_offset)
    # Enable output disable PAD_OD(bit 7)
    mem32[pad] |= (1 & 0x1) << 7
    # Do not enable input enable PAD_IE(bit 6)
    mem32[pad] |= (0 & 0x1) << 6
    mem32[gpio_ctrl] |= (GPIO_FUNC_NULL & 0x1f) << 0
    
def adc_init():
    mem32[ADC_CS]  = 0
    mem32[ADC_FCS] = 0
    mem32[ADC_CS] |= (1 & 0x1) << 0 # ADC_CS_EN (Bit 0) = 1
    
def adc_select_input(gpio):
    mux = gpio % 26
    mem32[ADC_CS] |= (mux & 0x07) << 12 # ADC_CS_AINSEL (bits 12:14)
    
def adc_set_round_robin(mask):
    mem32[ADC_CS] |= (mask & 0x1f) << 16 # ADC_CS_RROBIN (bits 16:20)
    
#def adc_fifo_setup(en, dreq_en, dreq_thresh, err_in_fifo, byte_shift):
def adc_fifo_setup():
    # Allow conversion result to be written into FIFO (instead of RESULT)
    mem32[ADC_FCS] |= (1 & 0x1) << 0 #mem32[ADC_FCS] |= (en & 0x1) << 0 # ADC_FCS_EN (bit 0)
    # Assert a DMA request when FIFO contains data (for transfer into a buffer)
    mem32[ADC_FCS] |= (1 & 0x1) << 3 #mem32[ADC_FCS] |= (dreq_en & 0x1) << 3 # ADC_FCS_DREQ_EN (bit 3)
    # Assert DREQ/IRQ when there is n bytes of data in the FIFO
    mem32[ADC_FCS] |= (0x1 & 0x0f) << 24 #mem32[ADC_FCS] |= (dreq_thresh & 0x0f) << 24 # ADC_FCS_THRESH (bits 24:27)
    # Do not include error bits in conversion result
    mem32[ADC_FCS] |= (0 & 0x1) << 2 #mem32[ADC_FCS] |= (err_in_fifo & 0x1) << 2 # ADC_FCS_ERR (bit 2)
    # Do not right shift FIFO results
    mem32[ADC_FCS] |= (0 & 0x1) << 1 #mem32[ADC_FCS] |= (byte_shift & 0x1) << 1 # ADC_FCS_SHIFT (bit 1)
    # Clear "underflowed FIFO" flag
    mem32[ADC_FCS] |= (1 & 0x1) << 10 # ADC_FCS_UNDER (bit 10) = 1
    # Clear "overflowed FIFO" flag
    mem32[ADC_FCS] |= (1 & 0x1) << 11 # ADC_FCS_OVER (bit 10) = 1
    
def adc_run(run):
    mem32[ADC_CS] |= (run & 0x1) << 3 # ADC_CS_START_MANY(bit3)
    
def adc_fifo_get_level():
    return (mem32[ADC_FCS] >> 16) & 0x0f
    
def adc_fifo_drain():
    x = 0
    while adc_fifo_get_level():
        x = mem32[ADC_IRQ_FIFO]
        #print(x)
    
    
    
adc_set_clkdiv(0)
mask = 0
for adc_pin in adc_pins:
    adc_gpio_init(adc_pin)
    mux = adc_pin % 26
    mask |= (1 & 0x1) << mux

#print(bin(mask))
adc_init()
adc_select_input(adc_pins[0])
adc_set_round_robin(mask)

"""
# Checks if ADC is working
mem32[ADC_CS] |= (1 & 0x1) << 2     # ADC_CS_START_ONCE (bit 2) = 1
print(mem32[ADC_RESULT])
"""

adc_fifo_setup()
#adc_fifo_setup(True,
#               True,
#               1,
#               False,
#               True
#)
adc_fifo_drain()



#############################
######  DMA Registers  ######
#############################
DMA_BASE           = 0x50000000
DMA_CHANNEL_WIDTH  = 0x40
DMA_CHANNEL_COUNT  = 12

# 16 bit mask that will abort an in-progress transfer sequence
# on those channels
DMA_CHAN_ABORT = DMA_BASE | 0x444

DMA_SIZE_8 = 0
DMA_SIZE_16 = 1
DMA_SIZE_32 = 2

DREQ_PWM_WRAP0  = 24
DREQ_PWM_WRAP1  = 25
DREQ_PWM_WRAP2  = 26
DREQ_PWM_WRAP3  = 27
DREQ_PWM_WRAP4  = 28
DREQ_PWM_WRAP5  = 29
DREQ_PWM_WRAP6  = 30
DREQ_PWM_WRAP7  = 31
DREQ_ADC        = 36
DREQ_DMA_TIMER0 = 59
DREQ_DMA_TIMER1 = 60
DREQ_DMA_TIMER2 = 61
DREQ_DMA_TIMER3 = 62
DREQ_FORCE      = 63

class DMA:
    def __init__(self, dma_channel_number, num_samples):
        self.DMA_CHANNEL_NUMBER = dma_channel_number
        self.DMA_CHANNEL_OFFSET = self.DMA_CHANNEL_NUMBER * DMA_CHANNEL_WIDTH
        self.num_samples = num_samples
        self.buffer = array.array('H', (0 for _ in range(self.num_samples)))
        
        self.DMA_READ_ADDR   = DMA_BASE | (0x000 + self.DMA_CHANNEL_OFFSET)
        self.DMA_WRITE_ADDR  = DMA_BASE | (0x004 + self.DMA_CHANNEL_OFFSET)
        self.DMA_TRANS_COUNT = DMA_BASE | (0x008 + self.DMA_CHANNEL_OFFSET)
        self.DMA_CTRL_TRIG   = DMA_BASE | (0x00c + self.DMA_CHANNEL_OFFSET)
        self.DMA_ALN_CTRL    = DMA_BASE | (0x010 + self.DMA_CHANNEL_OFFSET)
        #print(hex(self.DMA_CTRL_TRIG))
        
        self.dma_channel_abort()
        self.dma_channel_reset()
        self.dma_channel_set_read_addr(0)
        self.dma_channel_set_write_addr(0)
        self.dma_channel_set_trans_count(0)
        self.channel_config_set_chain_to(self.DMA_CHANNEL_NUMBER)
    
    def dma_channel_abort(self):
        mem32[DMA_CHAN_ABORT] |= (1 & 0x1) << self.DMA_CHANNEL_NUMBER
        while mem32[DMA_CHAN_ABORT] & (1 << self.DMA_CHANNEL_NUMBER):
            pass
        
    def dma_channel_reset(self):
        mem32[self.DMA_CTRL_TRIG] = 0
        
    def dma_channel_start(self):
        self.dma_channel_abort()
        mem32[self.DMA_CTRL_TRIG] |= (1 & 0x1) << 0
        
    def dma_channel_set_read_addr(self, read_addr):
        mem32[self.DMA_READ_ADDR] = read_addr
        
    def dma_channel_set_write_addr(self, write_addr):
        #print(hex(write_addr))
        mem32[self.DMA_WRITE_ADDR] = write_addr
    
    def dma_channel_set_trans_count(self, trans_count):
        mem32[self.DMA_TRANS_COUNT] = trans_count
        
    def channel_config_set_read_increment(self, incr):
        mem32[self.DMA_CTRL_TRIG] |= (incr & 0x1) << 4
        
    def channel_config_set_write_increment(self, incr):
        mem32[self.DMA_CTRL_TRIG] |= (incr & 0x1) << 5
        
    def channel_config_set_dreq(self, dreq):
        mem32[self.DMA_CTRL_TRIG] |= (dreq & 0x3f) << 15
        
    def channel_config_set_chain_to(self, chain_to):
        mem32[self.DMA_CTRL_TRIG] |= (chain_to & 0x4) << 11
        
    def channel_config_set_transfer_data_size(self, dma_channel_transfer_size):
        mem32[self.DMA_CTRL_TRIG] |= (dma_channel_transfer_size & 0x3) << 2
        
    def channel_config_set_irq_quiet(self, irq_quiet):
        mem32[self.DMA_CTRL_TRIG] |= (irq_quiet & 0x1) << 21
    
    def channel_config_set_enable(self, en):
        mem32[self.DMA_CTRL_TRIG] |= (en & 0x1) << 0
        

#NSAMPLES = 3
#adc_buff = array.array('H', (0 for _ in range(NSAMPLES)))

adc_dma = DMA(0, 3)
adc_dma.channel_config_set_chain_to(adc_dma.DMA_CHANNEL_NUMBER)
adc_dma.channel_config_set_write_increment(1)
adc_dma.channel_config_set_irq_quiet(1)
adc_dma.channel_config_set_dreq(DREQ_ADC)
adc_dma.channel_config_set_transfer_data_size(DMA_SIZE_16)
adc_dma.dma_channel_set_read_addr(ADC_IRQ_FIFO)
adc_dma.dma_channel_set_write_addr(uctypes.addressof(adc_dma.buffer))
adc_dma.dma_channel_set_trans_count(adc_dma.num_samples)
adc_dma.dma_channel_start()

pulse_dma = DMA(1, 1)
pulse_dma.buffer[0] = mem32[ADC_CS] | (1 & 0x1) << 3
pulse_dma.channel_config_set_chain_to(pulse_dma.DMA_CHANNEL_NUMBER)
pulse_dma.channel_config_set_write_increment(1)
pulse_dma.channel_config_set_irq_quiet(1)
pulse_dma.channel_config_set_dreq(DREQ_PWM_WRAP3)
pulse_dma.channel_config_set_transfer_data_size(DMA_SIZE_32)
pulse_dma.dma_channel_set_read_addr(uctypes.addressof(pulse_dma.buffer))
pulse_dma.dma_channel_set_write_addr(ADC_CS) #
pulse_dma.dma_channel_set_trans_count(pulse_dma.num_samples)
#pulse_dma.dma_channel_start()


# Need to capture the ADC enabled state
# and write that into the pulse_dma.buffer
# that way when DREQ_PWM_WRAP is signalled
# ADC goes into enabled state
#print(bin(mem32[ADC_CS]))
#adc_run(True)
#print(bin(mem32[ADC_CS]))
test_pwm = PWM(machine.Pin(7))
test_pwm.freq(20_000)
test_pwm.duty_u16(int(0.5 * 65536))
utime.sleep_ms(500)
pulse_dma.dma_channel_start()
#adc_run(False)
#utime.sleep_ms(500)
#print(bin(mem32[ADC_CS]))


#vals = [f"{val}" for val in adc_buff]
#utime.sleep_ms(500)
print(adc_dma.buffer)

