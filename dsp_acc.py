import numpy as np
import matplotlib.pyplot as plt
from scipy.fftpack import fft, ifft
import pandas as pd


def polarToRectangular(radii, angles):
    return radii * np.exp(1j * angles)



def frequencyGenerator(time, steps = 125):
    𝛿 = time.max() - time.min()
    M = np.arange(1, steps + 1)[:, np.newaxis]
    return M / 𝛿

def dFourT(time, values, frequency = None, steps = 125):
    if frequency is None:            
        ft = frequencyGenerator(time, steps)
        frequency = ft.reshape(ft.shape[0])
    else:
        ft = frequency[:, np.newaxis]
    
    # sorting the inputs
    order = np.argsort(time)
    ts = np.array(time)[order]
    Xs = np.array(values)[order]

    𝜃 = (ts - time.min()) * 2 * np.pi * ft
    Y = polarToRectangular(Xs, 𝜃)[:, :-1] * np.diff(ts)
    amplitude = np.abs(Y.sum(axis=1))
    return frequency, amplitude

def max_vals(filtered_time_series, filtered_freq_series):
    return
    