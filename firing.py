#installing packages for sensor https://github.com/adafruit/Adafruit_CircuitPython_AMG88xx
import busio
import board
import adafruit_amg88xx
import time
from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

#set up pins for DC motor and plunger
DC_EN=23
GPIO.setup(DC_EN, GPIO.OUT)
P_EN=19
GPIO.setup(P_EN, GPIO.OUT)

#initilize I2C bus
i2c_bus = busio.I2C(board.SCL, board.SDA)

#constants
threshold=30
min_diff=2

def aim_x(grid):
    x_aimed=0
    if x_flag==0:
        for i in range(8):
            if(grid[i][3]>threshold and grid[i][4]>threshold and abs(grid[i][3]-grid[i][4])<min_diff):
                x_aimed=1
    return x_aimed

fired=0
while fired==0:
    sensor = adafruit_amg88xx.AMG88XX(i2c_bus)
    print(sensor.pixels)
    if x_aimed(sensor.pixels):
        GPIO.output(P_EN,GPIO.HIGH)
        sleep(0.1)
        GPIO.output(DC_EN, GPIO.HIGH)
        GPIO.output(P_EN,GPIO.LOW)
        sleep(3)
        fired=1
GPIO.output(DC_EN,LOW)

GPIO.cleanup()
