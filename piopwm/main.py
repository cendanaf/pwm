from piopwm import PIOPWM
import machine
import utime
import rp2

time_resolution = 10 # bits
time_slices = (2 ** time_resolution) - 1

pwm = PIOPWM(0, 17, 26, max_count=time_slices, count_freq=25_650_000)

#duty = int(0.2 * time_slices)
#pwm.Set(duty)

i = int(0.2 * time_slices)
pwm.Set(i)

#i = 0
try:
    while True:
        adc = pwm.GetADCvalue()
        print(f"{i}, {adc}")
        #i += 10
        #i %= int(0.9 * (time_slices + 1))
        #pwm.Set(i)
        utime.sleep_ms(1000)
except KeyboardInterrupt:
    print("Ending program")
    pwm.End()
finally:
    print("Program end")


