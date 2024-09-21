from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio, asm_pio_encode
import utime
import time

@asm_pio(sideset_init=(PIO.OUT_LOW, PIO.OUT_LOW))
def PWM():
    pull(noblock)  .side(0b00)
    mov(x, osr) 
    mov(y, isr) 
    
    label("pwmloop1")
    jmp(x_not_y, "skip1")
    nop()          .side(0b01)
    
    label("skip1")
    jmp(y_dec, "pwmloop1")
    
    irq(0)         .side(0b11) 
    mov(y, x)     
    
    label("stage2")
    nop() [0]        # Use this for adjusting sampling point
    jmp(y_dec, "stage2")    
    

class PIOPWM:
    def __init__(self, sm_id, pwm_pin, adc_pin, max_count, count_freq):
        self._marker = Pin(0, Pin.OUT)
        self._adc = ADC(adc_pin)
        self._adc_value = 0
        self._count_freq = count_freq
        self._max_count = max_count
        self._PWMsm = StateMachine(sm_id,
                                   PWM,
                                   freq=2 * count_freq,
                                   sideset_base=Pin(pwm_pin))
        self._PWMsm.irq(handler=self.Sample)
        # Use exec() to load max count into ISR
        self._PWMsm.put(max_count)
        self._PWMsm.exec("pull()")
        self._PWMsm.exec("mov(isr, osr)")
        self._PWMsm.active(1)
        
    def Set(self, value):
        # Minimum value is -1 (completely turn off), 0 actually still produces narrow pulse
        value = max(value, -1)
        value = min(value, self._max_count)
        value = int((value-62) / 2)
        self._PWMsm.put(value)
        
    def Sample(self, sm):
        #print("Sampling")
        self._adc_value = self._adc.read_u16()
        
    def GetADCvalue(self):
        return self._adc_value
        
    def End(self):
        self._PWMsm.active(0)
        