import time
from machine import Pin, PWM, mem32
import micropython
import rp2

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW)
def MeasureHighTime():
# Calibration loop
    label("measure")
    mov(x, invert(null))        
    
    wait(0, pin, 0)
    wait(1, pin, 0)     .side(1)

    label("high_loop")
    jmp(x_dec, "cont_high_loop")
    label("cont_high_loop")
    nop()               
    jmp(pin, "high_loop")

    mov(isr, x)         .side(0)
    push(noblock)
    
    wait(1, pin, 0)
    
    #jmp("measure")

# Processing loop
    #label("processing")
    #mov(isr, invert(null))            
    #push(block)
    #jmp("processing")
    #pull(block)
    
# Standby loop
#    label("standby")
#    nop()            #.side(1)
#    nop()            #.side(0)
#    jmp("standby")
 
# Main loop
    #label("mainloop")
    #pull(noblock)     .side(0)
    #mov(x, osr)
    #mov(y, isr)
    #label("sampling_loop")
    #jmp(x_not_y, "skip")
    #irq(0)            .side(1)
    #label("skip")
    #jmp(y_dec, "sampling_loop")
    #jmp("mainloop")
    

def SmAddr(sm_num):
    addr = mem32[ 0x502000d4 + ((sm_num >> 2)) + (24 * (sm_num & 3)) ]
    return addr

def Calibrate(sm, sm_address):
    while sm.rx_fifo() != 0:
        sm.get()
    jmp = sm_address + 0
    sm.exec(f"set(x, {jmp})")
    sm.exec(f"mov(pc, x)")
    
    high = 0xffffffff - sm.get()
    half = int(high / 2)
    duty = high
    duty = max(0, duty)
    duty = min(duty, 4095)
    print("")
    print(f"{high:6} {half:4} {duty*1/(2**11):.2f}")
    print("")
    
    if sm.tx_fifo() == 0:
        print(f"Inputting {half} into the TX FIFO")
        sm.put(half)
    
    jmp = sm_address + 9
    #sm.exec(f"set(x, {jmp})")
    #sm.exec(f"mov(pc, x)")
    while sm.rx_fifo() != 0:
        a = 0xffffffff - sm.get()
    print(f"{a}, {jmp}")
    

pin17 = Pin(17, Pin.IN, Pin.PULL_UP)
pin18 = Pin(18, Pin.OUT)
sm0 = rp2.StateMachine(0, MeasureHighTime, freq=125_000_000,  in_base=pin17, jmp_pin=pin17, sideset_base=pin18)
sm0_addr = SmAddr(0)
print(sm0_addr)
sm0.active(1)

pin17 = Pin(17, Pin.OUT)
pwm = PWM(pin17)
pwm.init(freq=20_000, duty_ns=10_000)

Calibrate(sm0, sm0_addr)
sm0.active(0)

while sm0.rx_fifo() != 0:
    a = 0xffffffff - sm0.get()
    print(a)

print("")
print("Program over")