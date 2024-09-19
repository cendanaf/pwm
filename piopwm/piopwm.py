from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio, asm_pio_encode
import utime
import time

@asm_pio(sideset_init=(PIO.OUT_LOW, PIO.OUT_LOW))
def PWM():
    pull(noblock)    .side(0b00)
    mov(x, osr) 
    mov(y, isr) 
    
    label("pwmloop1") # 1st stage turns side pin on
    jmp(x_not_y, "skip1")
    nop()         .side(0b11) 
    jmp("transition")
    
    label("skip1")
    jmp(y_dec, "pwmloop1")
    
    label("transition") # Transition stage (supposedly) doubles the value of x
    mov(x, x << 1)
    jmp("pwmloop2")
    
    label("pwmloop2") # 2nd stage raises flag (at midpoint)
    jmp(x_not_y, "skip2")
    nop()            .side(0b01)
    
    label("skip2")
    jmp(y_dec, "pwmloop2")


class PIOPWM:
    def __init__(self, sm_id, pwm_pin, adc_pin, max_count, count_freq):
        self._marker = Pin(0, Pin.OUT)
        self._adc = ADC(adc_pin)
        self._adc_value = 0
        self._half_pulse = 0
        self._count_freq = count_freq
        self._max_count = max_count
        self._sm = StateMachine(sm_id, PWM, freq=2 * count_freq, sideset_base=Pin(pwm_pin))
        #self._sm.irq(lambda pio: print(pio.irq().flags()))
        self._sm.irq(handler=self.Sample)
        # Use exec() to load max count into ISR
        self._sm.put(max_count)
        self._sm.exec("pull()")
        self._sm.exec("mov(isr, osr)")
        self._sm.active(1)
        
    def Set(self, value):
        # Minimum value is -1 (completely turn off), 0 actually still produces narrow pulse
        value = max(value, -1)
        value = min(value, self._max_count)
        self._half_pulse = int((value * 1_000_000) / (2 * self._count_freq) )
        self._sm.put(value)
        
    def Sample(self, sm):
        #self._marker.value(1)
        #t0 = time.time_ns()
        #utime.sleep_us(self._half_pulse)
        self._adc_value = self._adc.read_u16()
        #t = time.time_ns()
        #print(f"{t-t0}")
        #self._marker.value(0)
    
    def GetADCvalue(self):
        return self._adc_value
        
    def End(self):
        self._sm.active(0)