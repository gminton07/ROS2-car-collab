import time
import argparse

# Import SPI and MCP3008 library.
import RPi.GPIO as GPIO
import Adafruit_MCP3008
from Adafruit_GPIO.GPIO import RPiGPIOAdapter as Adafruit_GPIO_Adapter

# Software SPI configuration:
CLK  = 40       # pin 13 on 3008
MISO = 21       # pin 12 on 3008
MOSI = 19       # pin 11 on 3008
CS   = 26       # pin 10 on 3008
gpio_adapter = Adafruit_GPIO_Adapter(GPIO, mode=GPIO.BOARD)
mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI, gpio=gpio_adapter)

DEBUG = True
parser = argparse.ArgumentParser(description="Read AD pins on the MCP3008")
parser.add_argument('--times', type=int, default=5,
                    help='number of times to read the pin')
parser.add_argument('--delay', type=float, default=2,
                    help='seconds of delay between readings')
parser.add_argument('--AD_pin', type=int, default=0,
                    help='AD input pin to read')
args = parser.parse_args()

if (DEBUG):
   print(f'AD_pin: {args.AD_pin}\tTimes: {args.times}\tDelay: {args.delay}\n') 

i=0
while (i<args.times):
   print(f'Raw ADC Value: {mcp.read_adc(args.AD_pin)}')
   time.sleep(args.delay)
   i=i+1
