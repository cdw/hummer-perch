#!/usr/bin/env python
# encoding: utf-8
"""
sumsines.py - drive an servo with a sum of sines over serial via arduino

Created by Dave Williams on 2014-01-29
"""

import sys, os
import time
import warnings
import numpy as np
from numpy import cos, sin, radians
import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt

TIME = 0.002    # Seconds between updates, temporal resolution
POS_MAX = 2200
POS_MIN =  700
TOT_ANG =  radians(60) # THIS IS A GUESS, VERIFY

def list_serial_ports():
    """ List all serial ports"""
    ports = []
    if os.name == 'nt':
        #Windows
        for i in range(256):
            try:
                s = serial.Serial(i)
                s.close()
                ports.append('COM' + str(i+1))
            except serial.SerialException:
                pass
    else:
        #Mac/unix
        for port in list_ports.comports():
            ports.append(port[0]) 
    return ports
    

def create_connection(port = '/dev/tty.usbmodemfd111'):
    ser = serial.Serial(port, baudrate = 115200) 
    return ser

def angle_conversion(angle):
    if angle>TOT_ANG:
        warnings.warn("Angle exceeds max: %i. Coercing."%np.degrees(TOT_ANG))
        angle = TOT_ANG
    if angle<0:
        warnings.warn("Angle exceeds min: 0. Coercing.")
        angle = 0
    return (POS_MAX-POS_MIN)*(angle/TOT_ANG)+POS_MIN

def write(ser, angle):
    ser.write('%i;'%angle_conversion(angle))

def sinewave(hz = 0.5, phase = 0, dur = 5):
    """Produce a sine wave oscillating between -1 and 1
    Takes
        hz: the frequency of the wave
        phase: phase offset in radians (0 to 2*pi)
        dur: the duration, in seconds
    Returns
        wave: the sine wave as a numpy array
    """
    t = np.arange(0, int(dur/TIME)) # t is measured in updates, not seconds
    omega = 2*np.pi*hz*TIME # TIME used to convert oscil/sec to oscil/update
    wave = sin(omega*t + phase)
    return wave

def sumofsines(hzs = [0.1, 0.5, 1.0], phases = [0, 0, 0], amps=[.1, 0.8, 0.1], dur = 20):
    """Produce a sum of sines for the specified wave parameters
    Takes
        hzs: wave frequencies
        phases: phase offsets in radians
        amps: amplitudes of sine waves
        dur: the duration, in seconds
    Returns
        wave: the summed sine waves as a numpy array
    """
    waves = [a*sinewave(h,p,dur) for h,p,a in zip(hzs, phases, amps)]
    return np.sum(waves, 0)/sum(amps)


def scale_wave(wave, center=TOT_ANG/2, envelope_amp = TOT_ANG):
    """Scale a wave assumed to be between -1 and 1
    Takes
        wave: unscaled waveform
        center: new y axis center of wave
        envelope_amp: new peak-to-peak amplitude
    Returns
        wave: scaled
    """
    return wave*0.5*envelope_amp + center

def write_wave(ser, wave):
    """Write a wave to the servo at the specified rate"""
    tick = time.time()
    for angle in wave:
        while time.time() - tick < TIME:
            time.sleep(TIME/1000)
        tick = time.time()
        write(ser, angle)

def single_sided_fft(wave, Fc=10, dt=TIME):
    """This plots the fast fourier transform of a waveform (and kinda provides
    an example of how to do so)
    Takes:
        Fc: the cuttoff frequency, don't display freqs above it
        dt: the time resolution (seconds between adjacent samples)
    """
    x = wave-np.mean(wave)  #remove dc offset
    Fs = 1.0/dt             #sample rate/freq, samp/sec
    N = len(x)              #sample length, number of samples
    freqs = scipy.fftpack.fftfreq(x.size, dt)
    freqs = freqs[freqs[:len(freqs)/2-1]<Fc] # cut off freqs, 0 to Fc
    fft = (abs(scipy.fft(x))/N)[:len(freqs)] #normed truncated fft
    return fft, freqs
