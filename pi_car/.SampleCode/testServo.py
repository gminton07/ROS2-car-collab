# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# 
# Modified : jdf 2/11/25 

# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.

import board
from adafruit_pca9685 import PCA9685

import time
import argparse

# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 60

parser = argparse.ArgumentParser(description="Test a servo with different PWM values")
parser.add_argument('--servo', type=int, default=0,
                    help='Servo channel on PCA9685 to test')
parser.add_argument('--value', type=int, default=0x1800,
                    help='Start value of servo in 16 digit hex')
args = parser.parse_args()


# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.

servo_value = args.value

while (servo_value > 0):
   pca.channels[args.servo].duty_cycle = servo_value
   servo_value = int(input("Enter next value to test, 0 to stop: "))

pca.reset()
pca.deinit()
