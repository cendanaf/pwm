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

# === GPIO =============================
GPIO_BASE    = 0xd0000000
GPIO_IN      = 0x004 + GPIO_BASE
GPIO_OUT     = 0x010 + GPIO_BASE
GPIO_OUT_SET = 0x014 + GPIO_BASE
GPIO_OUT_CLR = 0x018 + GPIO_BASE
GPIO_OE      = 0x020 + GPIO_BASE
GPIO_OE_SET  = 0x024 + GPIO_BASE
GPIO_OE_CLR  = 0x028 + GPIO_BASE



IOreg_write(GPIO_IN, 1<<lsc)      # gpio_set_dir(lsc, GPIO.IN)
IOreg_write(GPIO_OUT_CLR, 1<<lsc) # gpio_put(lsc, 0)
IOreg_write(GPIO_CTRL(lsc), 5)    # gpio_set_function(lsc, 5)

IOreg_write(GPIO_OE, 1<<lsc)      # gpio_set_dir(lsc, GPIO.OUT)

IOreg_write(GPIO_OUT_SET, 1<<lsc) # gpio_put(lsc, 1)



IOreg_write(GPIO_IN, 1<<lsb)      
IOreg_write(GPIO_OUT_CLR, 1<<lsb)
IOreg_write(GPIO_CTRL(lsb), 5)    

IOreg_write(GPIO_OE, 1<<lsb)      

IOreg_write(GPIO_OUT_SET, 1<<lsb)



IOreg_write(GPIO_IN, 1<<lsa)      
IOreg_write(GPIO_OUT_CLR, 1<<lsa)
IOreg_write(GPIO_CTRL(lsa), 5)    

IOreg_write(GPIO_OE, 1<<lsa)      

IOreg_write(GPIO_OUT_SET, 1<<lsa)




"""
# === IO =============================
IO_base   = 0x40014000
PADS_BASE = 0x4001c000

def IO_CTRL(chan_num):
    return chan_num*8 + 4 + IO_base

def IO_STATUS(chan_num):
    return chan_num*8 + GPIO_base

def IO_PAD_CTRL(pad_num):
    return (pad_num*4 + 4) + PADS_BASE

def GPIO2SliceNum(chan_num):
    return (chan_num >> 1) & 7
    

# ====================================
# GPIO setup 
SIO_FN = 5

IOreg_write(GPIO_CTRL(lsa), SIO_FN) 
IOreg_write(GPIO_CTRL(lsb), SIO_FN) 
IOreg_write(GPIO_CTRL(lsc), SIO_FN)




IOreg_write(PWM_EN, pwm_mask) # enable PWM simultaneously


# Output disable, input disable, PUE and PDE disable
IOreg_write(GPIO_PAD_CTRL(26), 0b10000000)
#IOreg_write(GPIO_PAD_CTRL(27), 0b10000000)
#IOreg_write(GPIO_PAD_CTRL(28), 0b10000000)
"""


