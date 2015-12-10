#!/usr/bin/python

#import sys, struct, time, os, datetime
from pymavlink import mavutil
#from MAVProxy.modules.lib import mavmemlog
from MAVProxy.modules.lib import grapher
#from MAVProxy.modules.lib import rline
#import math, re
#import numpy
from MAVProxy.modules.lib.wxgrapheditor import GraphDefinition, GraphDialog
from pylab import plot, show, title, xlabel, ylabel, subplot
from scipy import fft, arange

timeshift = 0
mlog = mavutil.mavlink_connection('55.log', notimestamps=False, zero_time_base=False)
#mavlog = mavmemlog.mavmemlog(mlog)
mg = grapher.MavGraph()
mg.axes = [1];
mg.field_types = [set(['IMU'])]
mg.fields = ['IMU.AccZ']
mg.first_only = [False]
mg.msg_types = set(['IMU'])
mg.multi = [False]
mg.timeshift = [0]
mg.x = [[]]
mg.y = [[]]
mg.add_mav(mlog)
mg.process_mav(mlog, timeshift)

sample_freq = 1/((mg.x[0][11]-mg.x[0][10])*1000) # sample frequency
n = len(mg.y[0]) # # of samples
T_interval = n/sample_freq #period
k = arange(n) #frequency range
frq = k/T_interval
frq = frq[range(n/2)]

Y = fft(mg.y[0])/n
Y = Y[range(n/2)]

plot(frq, abs(Y), 'r')
xlabel('Hz')
ylabel('Y(freq)')
title('Vibration FFT')
show()
