import PySimpleGUI as sg 
import os.path
import Logger 
import numpy as np 
import matplotlib as plt 
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


#sg.theme 

#data display windows 
fig1 = plt.figure.Figure(figsize=(5,4), dpi=100)
t = np.arange(0, 3, .01)
fig1.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

fig2 = plt.figure.Figure(figsize=(5,4), dpi=100)
t = np.arange(0, 3, .01)
fig2.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

fig3 = plt.figure.Figure(figsize=(5,4), dpi=100)
t = np.arange(0, 3, .01)
fig3.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

fig4 = plt.figure.Figure(figsize=(5,4), dpi=100)
t = np.arange(0, 3, .01)
fig4.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))

#Quick-drawing helper tool
plt.use("TkAgg")
def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=1)
    return figure_canvas_agg


#define window layout
layout = [
    [ 
        [sg.Text("Choose a file: "), sg.Input(key="-IN2-" ,change_submits=True, font='Arial'), sg.FileBrowse(key="-IN-", font='Arial')]  
    ],
    [
        [sg.Text('Duration to record (s)', size=(17,1), font='Arial'), sg.InputText(key="lp1", size=(5,1))],
        [sg.Button("Start Recording", font='Arial')]
    ],
    [
            [sg.Canvas(key="-CANVAS1-"), sg.Canvas(key="-CANVAS3-")],
            [sg.Canvas(key="-CANVAS2-"), sg.Canvas(key="-CANVAS4-")]
    ],
    
]

#Building Window
window = sg.Window('Pump Data Logger', layout, size=(1400,1000), finalize=True, icon='logo_bar.ico', background_color='#abd158')


#draw data plots
draw_figure(window["-CANVAS1-"].TKCanvas, fig1)
draw_figure(window["-CANVAS2-"].TKCanvas, fig2)
draw_figure(window["-CANVAS3-"].TKCanvas, fig3)
draw_figure(window["-CANVAS4-"].TKCanvas, fig4)

#event definitions
while True:
    event, values = window.read()
    if event == "Start Recording":

        Logger.record_data(values["lp1"], values["-IN-"])
    if event == sg.WIN_CLOSED or event=="Exit":
        break
    