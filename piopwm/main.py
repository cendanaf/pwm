from piopwm import PIOPWM
import machine
import utime
import rp2

time_resolution = 10 # bits
time_slices = (2 ** time_resolution) - 1
# Create PIOPWM object used to fade LED
# (state machine 0, pin 25, max PWM count of 65535, PWM freq of 10 MHz)
pwm = PIOPWM(0, 17, 26, max_count=time_slices, count_freq=20_510_000)

#duty = int(0.2 * time_slices)
#pwm.Set(duty)


i = 0

try:
    while True:
        adc = pwm.GetADCvalue()
        print(f"{adc}")
        i += 10
        i %= int(0.9 * (time_slices + 1))
        pwm.Set(i)
        utime.sleep_ms(1000)
except KeyboardInterrupt:
    print("Ending program")
    pwm.End()
finally:
    print("Program end")
    

