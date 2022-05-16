import os
import csv
import numpy as np
import binascii
import math
import math, cmath
import scipy.io as sio
import numpy.linalg as nplg
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt


def isRotationMatrix(R):
    Rt = np.transpose(R)  #
    shouldBeIdentity = np.dot(Rt, R)  #
    I = np.identity(3, dtype=R.dtype)  #
    n = np.linalg.norm(I - shouldBeIdentity)  #
    return n < 1e-6


def rotationMatrixToAngles(R):
    assert (isRotationMatrix(R))  #

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])  #

    singular = sy < 1e-6  #

    if not singular:  #
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:  #
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)  #
        z = 0

    return np.array([x, y, z])

def rotationMatrixToEulerAngles(R):
    #R = np.zeros((3, 3), dtype=np.float64)
    #cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    #print('dst:', R)
    x = x*180.0/3.141592653589793
    y = y*180.0/3.141592653589793
    z = z*180.0/3.141592653589793
    return x,y,z
def eulerAnglesToRotationMatrix(x,y,z) :
    theta = np.zeros((3, 1), dtype=np.float64) #
    theta[0] = x
    theta[1] = y
    theta[2] = z
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
    R = np.dot(R_z, np.dot( R_y, R_x ))
    #rvecs = np.zeros((1, 1, 3), dtype=np.float64)
    #rvecs,_ = cv2.Rodrigues(R, rvecstmp)
    return R