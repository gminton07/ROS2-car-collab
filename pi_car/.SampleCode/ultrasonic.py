import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)     # Ignore warning for now
GPIO.setmode(GPIO.BOARD)    # Use physical pin numberin
 
TRIG = 23   # define input and output pins
ECHO = 24

# set up the input and output pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, False)
GPIO.setup(ECHO, GPIO.IN)

time.sleep(.5)              # let the sensor initialize
time_start = time.time()    

# trigger a reading
GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG, False)

# find the start and end of the ultrasonic pulse
while GPIO.input(ECHO) == 0:
    start_time = time.time()
while GPIO.input(ECHO) == 1:
    end_time   = time.time()
time_end = time.time()

# Speed of sound 34300 cm/sec
total_distance = (end_time - start_time) * 34300
# Divide by 2, account for return trip for signal
print (f'Distance Away: {total_distance/2:.2f} cm')
print (f'Taken in {time_end-time_start:.3f} seconds')

GPIO.cleanup()              # reset the GPIO 
