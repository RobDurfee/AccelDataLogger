import PySimpleGUI as sg 
import os.path
import Logger 
import numpy as np 
from numpy import genfromtxt
#import matplotlib as plt 
import sys
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy.core.multiarray
import smbus
import RPi.GPIO as GPIO
import time, os
from time import time
from signal import signal, SIGINT, SIGTERM
from sys import exit
from dsp_acc import *

import csv 

###########################################################################
                        #   HARDWARE SETUP   #
###########################################################################

#Set LIS331 address
addr_x7 = 0x19
addr_base = 0x18

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

###########################################################################
                        #   SENSOR DATA CAPTURE   #
###########################################################################

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
def cleanup(signal_received, frame, allData):
        allData.close()
        #alrtData.close()
        GPIO.cleanup()
        exit(0)

######### MAIN CAPTURE FUNCTION #########

def record_data(duration, filepath, allData, sensor_base_bool):
    #Run this program unless there is a keyboard interrupt
    signal(SIGINT, cleanup)
    signal(SIGTERM, cleanup)
    print ("Starting stream")
    milli_init = int(time()*1000)

    #initialize LIS331 accelerometer
    initialize(addr_x7, 24)
    initialize(addr_base, 24)

    #Establish steady state reading 
    sensor_init_buffer = 50 #wait some ms for the sensors to init to avoid zero-ing off a spike
    milliseconds = int(time() * 1000) - milli_init
    while (milliseconds < sensor_init_buffer):

        milliseconds = int(time() * 1000) - milli_init
        initial_xAccl_x7, initial_yAccl_x7, initial_zAccl_x7 = readAxes(addr_x7)
        initial_xAccl_base, initial_yAccl_base, initial_zAccl_base = readAxes(addr_base) 

        initial_xAccl_x7, initial_yAccl_x7, initial_zAccl_x7 = convertToG(maxScale, initial_xAccl_x7, initial_yAccl_x7, initial_zAccl_x7)
        initial_xAccl_base, initial_yAccl_base, initial_zAccl_base = convertToG(maxScale, initial_xAccl_base, initial_yAccl_base, initial_zAccl_base) 

    milliseconds = int(time() * 1000) - milli_init


    ###### X7 sensor x*-1 --> spot z
    ###### X7 sensor z --> spot x 
    ###### X7 sensor y --> spot y 

    if (sensor_base_bool):
        ###### Sensor on base bracket, sensor x*-1 --> spot z
        ######                         sensor z --> spot x
        ######                         sensor y --> spot y 
        
        while milliseconds < ((int(duration)*1000) + sensor_init_buffer):
            
            milliseconds = int(time() * 1000) - milli_init
            #Get acceleration data for x, y, and z axes
            xAccl_x7, yAccl_x7, zAccl_x7 = readAxes(addr_x7)
            xAccl_base, yAccl_base, zAccl_base = readAxes(addr_base)
            
            #Calculate G force based on x, y, z acceleration data
            x_x7, y_x7, z_x7 = convertToG(maxScale, xAccl_x7, yAccl_x7, zAccl_x7)
            x_base, y_base, z_base = convertToG(maxScale, xAccl_base, yAccl_base, zAccl_base)

            #Write all sensor data to file chosen at start of program
            allData.write(str(milliseconds/1000) + "," + str(x_x7 - initial_xAccl_x7) + "," + str(y_x7 - initial_yAccl_x7) + "," + str(z_x7 - initial_zAccl_x7) + "," + str(x_base - initial_xAccl_base) + "," + str(y_base-initial_yAccl_base) + "," + str(z_base - initial_zAccl_base) + "\n")
        allData.close()
        print("data logged")
    else: 
        ###### Sensor on base bracket, sensor x*-1 --> spot z
        ######                         sensor z --> spot x
        ######                         sensor y --> spot y 
        
        while milliseconds < ((int(duration)*1000) + sensor_init_buffer):
            
            milliseconds = int(time() * 1000) - milli_init
            #Get acceleration data for x, y, and z axes
            xAccl_x7, yAccl_x7, zAccl_x7 = readAxes(addr_x7)
            xAccl_base, yAccl_base, zAccl_base = readAxes(addr_base)
            
            #Calculate G force based on x, y, z acceleration data
            x_x7, y_x7, z_x7 = convertToG(maxScale, xAccl_x7, yAccl_x7, zAccl_x7)
            x_base, y_base, z_base = convertToG(maxScale, xAccl_base, yAccl_base, zAccl_base)

            #Write all sensor data to file chosen at start of program
            allData.write(str(milliseconds/1000) + "," + str(x_x7 - initial_xAccl_x7) + "," + str(y_x7 - initial_yAccl_x7) + "," + str(z_x7 - initial_zAccl_x7) + "," + str(x_base - initial_xAccl_base) + "," + str(y_base-initial_yAccl_base) + "," + str(z_base - initial_zAccl_base) + "\n")
        allData.close()
        print("data logged")
    return



###########################################################################
                        #   DISPLAY SETUP   #
###########################################################################
def plot_data(allData):
    acm_data = genfromtxt(allData, delimiter=',', names="time, x_x7, y_x7, z_x7, x_base, y_base, z_base")
    a_x_x7 = acm_data["x_x7"]
    a_y_x7 = acm_data["y_x7"]
    a_z_x7 = acm_data["z_x7"]
    a_x_base = acm_data["x_base"]
    a_y_base = acm_data["y_base"]
    a_z_base = acm_data["z_base"]
    t = acm_data["time"]

    x_freq_x7, x_amp_x7 = dFourT(t, a_x_x7)
    x_freq_base, x_amp_base = dFourT(t, a_x_base)

    x_axis_fig = plt.figure()
    plt.plot(t, a_x_x7, label='X7 Mount')
    plt.plot(t, a_x_base, label='Spot Mount')
    plt.grid()
    plt.legend()
    plt.title('X-Axis Acceleration')
    plt.ylabel('X acceleration')
    plt.xlabel('Time (s)')

    y_axis_fig = plt.figure()
    plt.plot(t, a_y_x7, label='X7 Mount')
    plt.plot(t, a_y_base, label='Spot Mount')
    plt.grid()
    plt.legend()
    plt.title('Y-Axis Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Y acceleration')
   
    z_axis_fig = plt.figure()
    plt.plot(t, a_z_x7, label='X7 Mount')
    plt.plot(t, a_z_base, label='Spot Mount')
    plt.grid()
    plt.legend()
    plt.title('Z-Axis Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Z acceleration')

    
    x_freq_fig = plt.figure() 
    plt.plot(x_freq_x7, x_amp_x7, '-')
    plt.grid()
    plt.title('X-Axis FFT')
    plt.xlabel('Freq (Hz)')
    plt.ylabel('G/Hz')
    

    draw_figure(window["-CANVAS1-"].TKCanvas, x_axis_fig)
    draw_figure(window["-CANVAS2-"].TKCanvas, y_axis_fig)
    draw_figure(window["-CANVAS3-"].TKCanvas, z_axis_fig)
    draw_figure(window["-CANVAS4-"].TKCanvas, x_freq_fig)
    return

#Quick-drawing helper tool
def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return figure_canvas_agg


#define window layout
layout = [
    [ 
        [sg.Text("Choose a file:"), sg.Input(key="-IN2-" ,change_submits=True, font='Arial'), sg.FileBrowse(key="-IN-", font='Arial')],
        [sg.Text('Recording Time (s)', size=(15,1), font='Arial'), sg.InputText(key="lp1", size=(5,1))]  
    ],
    [
        [sg.Radio('Sensor On Base', "RADIO1", default=True, key="-RADIO1-"),
        sg.Radio('Sensor On Body', "RADIO1")],
        [sg.Button("Start Recording", font='Arial')]
        
    ],
    [
            [sg.Canvas(key="-CANVAS1-"), sg.Canvas(key="-CANVAS3-")],
            [sg.Canvas(key="-CANVAS2-"), sg.Canvas(key="-CANVAS4-")]
    ],
    
]

#Building Window
window = sg.Window('Pump Data Logger', layout, size=(1600,1000), finalize=True, icon='logo_bar.ico', background_color='#abd158')


###########################################################################
                        #   EVENT DEFINITIONS   #
###########################################################################



while True:
    event, values = window.read()
    if event == "Start Recording":
        allData = open(values["-IN2-"], "w")
        record_data(values["lp1"], values["-IN-"], allData, values["-RADIO1-"])
        #print(allData.name)
        plot_data(allData.name)
        print(values["-RADIO1-"])

    if event == sg.WIN_CLOSED or event=="Exit":
        break
    