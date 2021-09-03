import PySimpleGUI as sg 
import os.path
import Logger 
import numpy as np 
from numpy import genfromtxt
#import matplotlib as plt 
import sys
import os
import pandas as pd

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

###########################################################################
                        #   FILE WRITING   #
###########################################################################

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

    ###### X7 sensor x*-1 --> spot x7 z
    ###### X7 sensor z --> spot x7 x 
    ###### X7 sensor y --> spot x7 y 

    if (sensor_base_bool):
        ###### Sensor on base bracket, sensor x*-1 --> spot base z
        ######                         sensor z --> spot base x
        ######                         sensor y --> spot base y 
        
        while milliseconds < ((int(duration)*1000) + sensor_init_buffer):
            
            milliseconds = int(time() * 1000) - milli_init
            #Get acceleration data for x, y, and z axes
            xAccl_x7, yAccl_x7, zAccl_x7 = readAxes(addr_x7)
            xAccl_base, yAccl_base, zAccl_base = readAxes(addr_base)
            
            #Calculate G force based on x, y, z acceleration data
            x_x7, y_x7, z_x7 = convertToG(maxScale, xAccl_x7, yAccl_x7, zAccl_x7)
            x_base, y_base, z_base = convertToG(maxScale, xAccl_base, yAccl_base, zAccl_base)

            #Write all sensor data to file chosen at start of program
            # spot_x7_x, spot_x7_y, spot_x7_z, spot_base_x, spot_base_y, spot_base_z

            allData.write(str(milliseconds/1000) + "," + str(z_x7 - initial_zAccl_x7) + "," + str(y_x7 - initial_yAccl_x7) + "," + str((x_x7 - initial_xAccl_x7)*-1) + "," + str(z_base - initial_zAccl_base) + "," + str(y_base-initial_yAccl_base) + "," + str((x_base - initial_xAccl_base)*-1) + "\n")
        allData.close()
        print("data logged")
    else: 
        ###### Sensor on body,         sensor x*-1 --> spot body z
        ######                         sensor z --> spot body x
        ######                         sensor y --> spot body y 

        while milliseconds < ((int(duration)*1000) + sensor_init_buffer):
            
            milliseconds = int(time() * 1000) - milli_init
            #Get acceleration data for x, y, and z axes
            xAccl_x7, yAccl_x7, zAccl_x7 = readAxes(addr_x7)
            xAccl_base, yAccl_base, zAccl_base = readAxes(addr_base)
            
            #Calculate G force based on x, y, z acceleration data
            x_x7, y_x7, z_x7 = convertToG(maxScale, xAccl_x7, yAccl_x7, zAccl_x7)
            x_base, y_base, z_base = convertToG(maxScale, xAccl_base, yAccl_base, zAccl_base)

            #Write all sensor data to file chosen at start of program
            allData.write(str(milliseconds/1000) + "," + str(z_x7 - initial_zAccl_x7) + "," + str(y_x7 - initial_yAccl_x7) + "," + str((x_x7 - initial_xAccl_x7)*-1) + "," + str(z_base - initial_zAccl_base) + "," + str(y_base-initial_yAccl_base) + "," + str((x_base - initial_xAccl_base)*-1) + "\n")
        allData.close()
        print("data logged")
    return

###########################################################################
                #   ADD HEADERS TO CSV POST-CAPTURE   #
###########################################################################

def append_data_headers(filepath):
    file = pd.read_csv(filepath)
    headerlist = ['time', 'spot_x_axis_x7', 'spot_y_axis_x7', 'spot_z_axis_x7', 'spot_x_axis_base', 'spot_y_axis_base', 'spot_z_axis_base']
    file.to_csv(filepath, header=headerlist, index=False)
    

def append_data_headers_freq(filepath):
    file = pd.read_csv(filepath)
    headerlist = ['Hz', 'spot_x_axis_x7 (G/Hz)', 'spot_x_axis_base (G/Hz)', 'spot_y_axis_x7 (G/Hz)', 'spot_y_axis_base (G/Hz)', 'spot_z_axis_x7 (G/Hz)', 'spot_z_axis_base (G/Hz)']
    file.to_csv(filepath, header=headerlist, index=False)
    
###########################################################################
                        #   DISPLAY SETUP   #
###########################################################################
def plot_data(allData):
    acm_data = genfromtxt(allData, delimiter=',', names="time, spot_x_x7, spot_y_x7, spot_z_x7, spot_x_base, spot_y_base, spot_z_base")
    a_x_x7 = acm_data["spot_x_x7"]
    a_y_x7 = acm_data["spot_y_x7"]
    a_z_x7 = acm_data["spot_z_x7"]
    a_x_base = acm_data["spot_x_base"]
    a_y_base = acm_data["spot_y_base"]
    a_z_base = acm_data["spot_z_base"]
    t = acm_data["time"]

    spot_x_freq_x7, spot_x_amp_x7 = dFourT(t, a_x_x7)
    spot_x_freq_base, spot_x_amp_base = dFourT(t, a_x_base)

    spot_y_freq_x7, spot_y_amp_x7 = dFourT(t, a_y_x7)
    spot_y_freq_base, spot_y_amp_base = dFourT(t, a_y_base)

    spot_z_freq_x7, spot_z_amp_x7 = dFourT(t, a_z_x7)
    spot_z_freq_base, spot_z_amp_base = dFourT(t, a_z_base)

    x_axis_fig = plt.figure()
    plt.plot(t, a_x_x7, label='X7 Mount')
    plt.plot(t, a_x_base, label='Spot Mount')
    plt.grid()
    plt.legend()
    plt.title('Spot X-Axis Acceleration')
    plt.ylabel('Acceleration (G)')
    plt.xlabel('Time (s)')

    y_axis_fig = plt.figure()
    plt.plot(t, a_y_x7, label='X7 Mount')
    plt.plot(t, a_y_base, label='Spot Mount')
    plt.grid()
    plt.legend()
    plt.title('Spot Y-Axis Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Y acceleration (G)')
   
    z_axis_fig = plt.figure()
    plt.plot(t, a_z_x7, label='X7 Mount')
    plt.plot(t, a_z_base, label='Spot Mount')
    plt.grid()
    plt.legend()
    plt.title('Spot Z-Axis Acceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Z acceleration (G)')

    
    freq_fig, axs = plt.subplots(3) 
    freq_fig.suptitle('Frequency Domain Approximation')
 
    
    axs[0].plot(spot_x_freq_x7, spot_x_amp_x7, label='X7 Mount')
    axs[0].plot(spot_x_freq_base, spot_x_amp_base, label='Spot Mount')
    axs[0].set_title('X-Axis')

    axs[1].plot(spot_y_freq_x7, spot_y_amp_x7, label='X7 Mount')
    axs[1].plot(spot_y_freq_base, spot_y_amp_base, label='Spot Mount')
    axs[1].set_title('Y-Axis')

    axs[2].plot(spot_z_freq_x7, spot_z_amp_x7, label='X7 Mount')
    axs[2].plot(spot_z_freq_base, spot_z_amp_base, label='Spot Mount')
    axs[2].set_title('Z-Axis')


    for ax in freq_fig.get_axes():
        ax.legend()
        ax.grid()
    

    freq_data_total = zip(spot_x_freq_x7, spot_x_amp_x7, spot_x_amp_base, spot_y_amp_x7, spot_y_amp_base, spot_z_amp_x7, spot_z_amp_base)

    with open('Freq_Data.csv', 'w') as f: 
        writer = csv.writer(f)
        for row in freq_data_total:
            writer.writerow(row)
    
    


    append_data_headers_freq('Freq_Data.csv')

    canvas1 = draw_figure(window["-CANVAS1-"].TKCanvas, x_axis_fig)
    canvas2 = draw_figure(window["-CANVAS2-"].TKCanvas, y_axis_fig)
    canvas3 = draw_figure(window["-CANVAS3-"].TKCanvas, z_axis_fig)
    canvas4 = draw_figure(window["-CANVAS4-"].TKCanvas, freq_fig)
    return canvas1, canvas2, canvas3, canvas4


#Quick-drawing helper tool
def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return figure_canvas_agg

def delete_figure_agg(figure_agg):
    figure_agg.get_tk_widget().forget()
    plt.close('all')
    return

#define window layout

canvas_column = [
    [sg.Canvas(key="-CANVAS1-") , sg.Canvas(key="-CANVAS3-")],
    [sg.Canvas(key="-CANVAS2-"), sg.Canvas(key="-CANVAS4-")]
]  

controls_column = [ 
        [sg.Text("Choose a file:"), sg.Input(key="-IN2-" ,change_submits=True, font='Arial'), sg.FileBrowse(key="-IN-", font='Arial')],
        [sg.Text('Recording Time (s)', size=(15,1), font='Arial'), sg.InputText(key="lp1", size=(5,1)), sg.Radio('Sensor On Bracket', "RADIO1", default=True, key="-RADIO1-"), sg.Radio('Sensor On Body', "RADIO1"), sg.Button("Start Recording", font='Arial'), sg.Button("Clear Window", font='Arial')] 
]

layout = [
     
        
        
        [sg.Column(controls_column, justification='center', element_justification='center')],
        [sg.Column(canvas_column, justification='center', element_justification='center')]

    
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
        canvas1, canvas2, canvas3, canvas4 = plot_data(allData.name)
        append_data_headers(allData.name)
    if event == "Refresh Window":
        
        
        delete_figure_agg(canvas1)
        delete_figure_agg(canvas2)
        delete_figure_agg(canvas3)
        delete_figure_agg(canvas4)

        

    


    if event == sg.WIN_CLOSED or event=="Exit":
        break
    