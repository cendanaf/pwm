from machine import Pin, ADC
from rp2 import PIO, StateMachine, asm_pio
import utime

@asm_pio(sideset_init=PIO.OUT_LOW)
def PWM():
    pull(noblock) .side(0)
    mov(x, osr) # Keep most recent pull data stashed in X, for recycling by noblock
    mov(y, isr) # ISR must be preloaded with PWM count max
    label("pwmloop")
    jmp(x_not_y, "skip")
    irq(0)         .side(1)
    label("skip")
    jmp(y_dec, "pwmloop")


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
        self._marker.value(1)
        utime.sleep_us(self._half_pulse)
        self._adc_value = self._adc.read_u16()
        self._marker.value(0)
    
    def GetADCvalue(self):
        return self._adc_value
        
    def End(self):
        self._sm.active(0)