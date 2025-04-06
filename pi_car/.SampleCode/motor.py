import RPi.GPIO as GPIO          
GPIO.setmode(GPIO.BOARD)
from time import sleep
import mod7_func as motor

in1 = 12
in2 = 13
en  = 11

print ("Initialize Motor Duty Cycle 50%")
pwm_pin   = motor.motor_init(in1, in2, en, 1000, 50)
sleep(1)

print ("Set to move forward 3 seconds") 
motor.motor_direction(in1, in2, 1)
sleep(3)

print ("Set to move backward 3 seconds") 
motor.motor_direction(in1, in2, -1)
sleep(3)

print ("Set to move not move 3 seconds") 
motor.motor_direction(in1, in2, 0)
sleep(3)

print ("Set to move forward at DC of 5% for 3 seconds")
pwm_pin.ChangeDutyCycle(5)
motor.motor_direction(in1, in2, 1)
sleep(3)

print ("Move up by 2% every second until hitting 20")
for i in range(7, 21, 2):
   print (f"duty cycle: {i}") 
   pwm_pin.ChangeDutyCycle(i)
   sleep(1)

print ("Set to move forward at DC of 90% for 3 seconds")
pwm_pin.ChangeDutyCycle(90)
sleep(3)

print ("Set to move backwards at DC of 90% for 3 seconds")
motor.motor_direction(in1, in2, -1)
sleep(3)

GPIO.cleanup()
