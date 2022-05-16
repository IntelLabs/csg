
import os
import sys
import random
import math
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt
import pickle
import scipy.io as sio
from PIL import Image, ImageDraw
import pylab as plt
import numpy.linalg as nplg

# Root directory of the project
#ROOT_DIR = os.getcwd()

# Directory of images to run detection on
#IMAGE_DIR = os.path.join(ROOT_DIR, "images")
# Pmat=sio.loadmat('MatForTestPara_1900_190313.mat')
#Pmat = sio.loadmat('PMat_1900_190513_case2.mat')
#Pinst = Pmat['P_Test']
#Hinst = Pmat['H_Test']


def I2WCal(x, y, h_real, isI2W, Pinst):
    P = np.asarray(Pinst)
    H = P[:, (0, 2, 3)]
    if isI2W == 1:
        x1 = x
        y1 = y
        if h_real == 0:
            # print(H)
            # print(Tran)
            # print(Tran*(np.linalg.inv(H)*np.array([[x1,y1,1]]).T))
            Coor = normalization(np.dot(np.linalg.inv(H), np.array([x1, y1, 1]).reshape((3, 1))))
            # VA=np.dot(np.linalg.inv(H),np.array([x1,y1,1]).reshape((3,1)))
            # Coor=normalization(np.dot(Tran,np.dot(np.linalg.inv(H),np.array([x1,y1,1]).reshape((3,1)))))
            # print(VA)
        else:
            h = h_real
            Hi = [x1, y1]
            a1 = Hi[0] * P[2, 0] - P[0, 0]
            b1 = Hi[0] * P[2, 1] - P[0, 1]
            a2 = Hi[1] * P[2, 0] - P[1, 0]
            b2 = Hi[1] * P[2, 1] - P[1, 1]
            c1 = (P[0, 2] - Hi[0] * P[2, 2]) * h + P[0, 3] - Hi[0] * P[2, 3]
            c2 = (P[1, 2] - Hi[1] * P[2, 2]) * h + P[1, 3] - Hi[1] * P[2, 3]
            Hl = np.array([[a1, b1], [a2, b2]])
            C = np.array([[c1], [c2]])
            temp1 = np.dot(np.linalg.inv(Hl), C)
            # print((P[1,2]-Hi[1]*P[2,2])*h)
            # print(Hi[1]*P[2,3])
            # print(P[1,3])
            # print(Hl)
            # print(np.linalg.inv(Hl))
            # print(C)
            # Coor=np.array([temp1[0],temp1[1]])
            # Coor=normalization(np.dot(Tran,np.array([temp1[0],temp1[1],1]).reshape((3,1))))
            Coor = normalization(np.array([temp1[0], temp1[1], 1]).reshape((3, 1)))
            # Coor=normalization(temp1.reshape((3,1)))

    elif isI2W == 0:

        Coor = normalization(np.dot(P, np.array([x, y, h_real, 1]).reshape((4, 1))))
        Coor[0] = Coor[0] * 1
        Coor[1] = Coor[1] * 1
    # X=Coor[0]
    # Y=Coor[1]

    return Coor


# In[272]:

def normalization(x):
    num = x.shape[1]
    y = np.zeros((3, num));
    for i in range(0, num):
        y[0, i] = x[0, i] / x[2, i];
        y[1, i] = x[1, i] / x[2, i];
        y[2, i] = 1;
    return y


def BBoxRegression(CData, Outline, Class_type, ry):
    H = Hinst
    P = Pinst
    CData1 = np.zeros((1080, 1902))
    # CData1[Outline(0):Outline(2),Outline(1):Outline(3)]=CData;
    CData1 = CData
    # P=Pmat['P_'+CamNum]
    # Tran=TranMat['tran_'+CamNum]
    # HeightHeadCurve=HeightHeadCurveMat['HeightHeadCurve_'+CamNum]
    # HeightFootCurve=HeightFootCurveMat['HeightCurve_'+CamNum]
    # H=np.array(P[:,[0,1,3]])
    CenterInitial1 = np.array([(Outline[1] + Outline[3]) / 2, (Outline[0] + Outline[2]) / 2])
    # print(CenterInitial1)
    BIN = 3
    BinInter = 5
    XRange = np.arange(-(Outline[3] - Outline[1]) / BIN, (Outline[3] - Outline[1]) / BIN, BinInter)
    YRange = np.arange(-(Outline[2] - Outline[0]) / BIN, (Outline[2] - Outline[0]) / BIN, BinInter)
    CenterCandX = XRange + CenterInitial1[0]
    CenterCandY = YRange + CenterInitial1[1] + 10
    UniCenterCandX = CenterCandX * 1
    UniCenterCandY = CenterCandY * 1
    LengthX = len(XRange)
    LengthY = len(YRange)
    face_idx = np.array([[1, 2, 6, 5], [2, 3, 7, 6], [3, 4, 8, 7], [4, 1, 5, 8]])
    Dimension = np.zeros((9, 3))
    Dimension_Car = np.array([[4200, 1700, 1400]])
    Dimension_Truck = np.array([[6500, 3500, 2500]])
    Dimension[3, :] = Dimension_Car
    Dimension[6, :] = Dimension_Truck
    R = np.array([[math.cos(ry), -math.sin(ry), 0], [math.sin(ry), math.cos(ry), 0], [0, 0, 1]])
    l = Dimension[Class_type, 0]
    w = Dimension[Class_type, 1]
    h = Dimension[Class_type, 2]
    y_corners = np.array([[l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]])
    x_corners = np.array([[w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]])
    z_corners = np.array([[0, 0, 0, 0, h, h, h, h]])
    Dimension_3D = np.concatenate((x_corners, y_corners, z_corners), axis=0)
    Dimension_3D = np.dot(R, Dimension_3D)
    MountFinal = 0
    worldLocation = 0
    for i in range(0, LengthX):
        for j in range(0, LengthY):
            Coor = normalization(
                np.dot(np.linalg.inv(H), np.array([UniCenterCandX[i], UniCenterCandY[j], 1]).reshape((3, 1))))
            # temp=[Coor(1)/Coor(3),Coor(2)/Coor(3),0];
            corners_3D = np.zeros((3, 8))
            corners_3D[0, :] = Dimension_3D[0, :] + Coor[0]
            corners_3D[1, :] = Dimension_3D[1, :] + Coor[1]
            corners_3D[2, :] = Dimension_3D[2, :]
            # corners_3D(3,:)=corners_3D(3,:)*HeightFootCurve(floor(CenterCandY(j)));
            corners_2D = np.dot(P, np.concatenate((corners_3D, np.ones((1, corners_3D.shape[1]))), axis=0))
            corners_2D = normalization(corners_2D)
            corners_2D[0, :] = corners_2D[0, :] * 1
            corners_2D[1, :] = corners_2D[1, :] * 1
            if corners_2D[0, 0] > corners_2D[0, 7]:
                draw_order = np.array([1, 5, 6, 7, 3, 4, 1]) - 1
            else:
                draw_order = np.array([2, 3, 4, 8, 5, 6, 2]) - 1
                # print(corners_2D)
            NewCornersX = corners_2D[0, draw_order] - round(min(corners_2D[0, draw_order]))
            NewCornersY = corners_2D[1, draw_order] - round(min(corners_2D[1, draw_order]))
            NewRangeX = int(3 + round(max(corners_2D[0, draw_order]) - min(corners_2D[0, draw_order])))
            NewRangeY = int(3 + round(max(corners_2D[1, draw_order]) - min(corners_2D[1, draw_order])))
            Xs = int(max(round(min(corners_2D[0, draw_order])) - 10, 1))
            Ys = int(max(round(min(corners_2D[1, draw_order])) - 10, 1))
            # print(Xs,Ys)
            # print(NewRangeX,NewRangeY)
            CData1_Resize = CData1[Ys:min(Ys + NewRangeY, 1080), Xs:min(Xs + NewRangeX, 1902)]
            # mask = poly2mask(NewCornersX+10,NewCornersY+10,(min(Ys+NewRangeY,1080)-Ys,min(Xs+NewRangeX,1902)-Xs))
            mask = poly2mask(np.maximum(corners_2D[0, draw_order], 1), np.maximum(corners_2D[1, draw_order], 1),
                             [1080, 1902])
            # mask=poly2mask(NewCornersX+10,NewCornersY+10,min(Ys+NewRangeY,1200)-Ys,min(Xs+NewRangeX,1600)-Xs);
            # print('mask',mask.shape)
            Mount = mask * CData1

            Mount = np.sum(Mount)
            if MountFinal < Mount:
                # Location=[CenterCandX(i),CenterCandY(j)]
                worldLocation = Coor
                MountFinal = Mount
                # plt.imshow(Mount)
                # Corners_2D_Final=corners_2D
                # DrawOrder=draw_order
                MaskOut = mask

            else:
                MountFinal = MountFinal

    return worldLocation


# In[234]:


from skimage import draw
import numpy as np


def poly2mask(vertex_row_coords, vertex_col_coords, shape):
    fill_row_coords, fill_col_coords = draw.polygon(vertex_row_coords, vertex_col_coords, shape)
    mask = np.zeros(shape, dtype=np.bool)
    mask[fill_row_coords, fill_col_coords] = 1
    return mask

# In[ ]:
