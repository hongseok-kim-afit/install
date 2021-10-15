import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as la
from math import pi, cos, sin, sqrt, exp
import sys
import copy

# tracking coordinate change to depth coordinate
# t265 right: positive x, up: positive y, back: positive z
# d435, right: positive x, down: positive y, forward: positive z
# https://github.com/IntelRealSense/librealsense/tree/development/examples/tracking-and-depth
ext= np.array([[0.999968402, -0.006753626, -0.004188075, -0.015890727],
[-0.006685408, -0.999848172, 0.016093893, 0.028273059],
[-0.004296131, -0.016065384, -0.999861654, -0.009375589]]) 
print('extrinc:', ext)
a=[1,0,1,1]
b=ext.dot(a)
print('result',b)
it= np.array([[0.999968402, -0.006753626, -0.004188075],
[-0.006685408, -0.999848172, 0.016093893],
[-0.004296131, -0.016065384, -0.999861654]]) 

c=np.linalg.inv(it)
print('inv',c)
e=np.array(ext[:,3])
e=e.reshape((3,1))
print('e',e)

intrin=np.hstack((c,e))
print('intrinsic',intrin)
d=intrin.dot(a)
print('d435 to t265:',d)
'''
a=[1,0,1,1]
ext
result = [ 0.9798896   0.03768154 -1.01353337]

a=[1,1,1,1]
ext
result = [ 0.97313597 -0.96216663 -1.02959876]

a=[0,0,0,0]
result =[0. 0. 0.]

a=[0,0,0,1]
result =[-0.01589073  0.02827306 -0.00937559]
'''
