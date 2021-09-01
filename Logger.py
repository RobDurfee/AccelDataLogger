
import smbus
import numpy.core.multiarray
#GPIO
import RPi.GPIO as GPIO

#Other
import time, os
from time import time
from signal import signal, SIGINT, SIGTERM
from sys import exit

####################################
#        User Parameters
#    (Edit these as necessary)
####################################
#Set LIS331 address
addr = 0x19

#Set the acceleration range
maxScale = 24


#LIS331 Constants (see Datasheet)
CTRL_REG1 = 0x20
CTRL_REG4 = 0x23
OUT_X_L = 0x28
OUT_X_H = 0x29
OUT_Y_L = 0x2A
OUT_Y_H = 0x2B
OUT_Z_L = 0x2C
OUT_Z_H = 0x2D

POWERMODE_NORMAL = 0x27
RANGE_6G = 0x00
RANGE_12G = 0x10
RANGE_24G = 0x30


# Create I2C bus
bus = smbus.SMBus(1)
allData = open("AllSensorData.txt", "a")
#alrtData = open("AlertData.txt", "a")

#Initiliaze LIS331
def initialize(addr, maxScale):
    scale = int(maxScale)
    #Initialize accelerometer control register 1: Normal Power Mode and 50 Hz sample rate
    bus.write_byte_data(addr, CTRL_REG1, POWERMODE_NORMAL)
    #Initialize acceleromter scale selection (6g, 12 g, or 24g). This example uses 24g
    if maxScale == 6:
        bus.write_byte_data(addr, CTRL_REG4, RANGE_6G)
    elif maxScale == 12:
        bus.write_byte_data(addr, CTRL_REG4, RANGE_12G)
    elif maxScale == 24:
        bus.write_byte_data(addr, CTRL_REG4, RANGE_24G)
    else:
        print("Error in the scale provided -- please enter 6, 12, or 24")


#Function to read the data from accelerometer
def readAxes(addr):
    data0 = bus.read_byte_data(addr, OUT_X_L)
    data1 = bus.read_byte_data(addr, OUT_X_H)
    data2 = bus.read_byte_data(addr, OUT_Y_L)
    data3 = bus.read_byte_data(addr, OUT_Y_H)
    data4 = bus.read_byte_data(addr, OUT_Z_L)
    data5 = bus.read_byte_data(addr, OUT_Z_H)
    #Combine the two bytes and leftshit by 8
    x = data0 | data1 << 8
    y = data2 | data3 << 8
    z = data4 | data5 << 8
    #in case overflow
    if x > 32767 :
        x -= 65536
    if y > 32767:
        y -= 65536
    if z > 32767 :
        z -= 65536
    #Calculate the two's complement as indicated in the datasheet
    x = ~x
    y = ~y
    z = ~z
    return x, y, z

#Function to calculate g-force from acceleration data
def convertToG(maxScale, xAccl, yAccl, zAccl):
    #Caclulate "g" force based on the scale set by user
    #Eqn: (2*range*reading)/totalBits (e.g. 48*reading/2^16)
    X = (2*float(maxScale) * float(xAccl))/(2**16);
    Y = (2*float(maxScale) * float(yAccl))/(2**16);
    Z = (2*float(maxScale) * float(zAccl))/(2**16);
    return X, Y, Z


# closes files and GPIO
def cleanup(signal_received, frame):
        allData.close()
        #alrtData.close()
        GPIO.cleanup()
        exit(0)

####################################
#       Main Function
####################################
def record_data(duration, filepath):
    #Run this program unless there is a keyboard interrupt
    signal(SIGINT, cleanup)
    signal(SIGTERM, cleanup)
    print ("Starting stream")
    milli_init = int(time()*1000)
    while True:
        #initialize LIS331 accelerometer
        initialize(addr, 24)

        #Start timestamp
        milliseconds = int(time() * 1000) - milli_init
        
        #Get acceleration data for x, y, and z axes
        xAccl, yAccl, zAccl = readAxes(addr)
        
        #Calculate G force based on x, y, z acceleration data
        x, y, z = convertToG(maxScale, xAccl, yAccl, zAccl)

        #Write all sensor data to file AllSensorData (as you probably guessed :) )
        allData.write(str(milliseconds) + "," + str(x) + "," + str(y) + "," + str(z) + "\n")

    return




    