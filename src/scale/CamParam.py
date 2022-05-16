
import random
from .CamCal_config import Config
import numpy as np
import math
from .ParamRng import ParamRng
import secrets

class CamParam:
    def __init__(self):
        # self.R_Mat = []
        # self.Fx = 0
        # self.Fy = 0
        # self.Cx = 0
        # self.Cy = 0
        # self.Pitch = 0
        # self.Yaw = 0
        # self.Roll = 0
        # self.Tx = 0
        self.Ty = 0
        # self.Tz = 0
        self.Scale_X = 0
        self.Scale_Z = 0
        self.config = Config()
        self.img_width =1920
        self.img_height =1080
        self.Mat_K = np.zeros((3, 3))
        self.Mat_R = []
        self.Mat_T = []
        self.Mat_P = []
        self.fReprojErr=0

    def setMatrix_K(self,f_x, f_y, c_x, c_y):
        self.Mat_K[0, 0] = f_x
        self.Mat_K[1, 1] = f_y
        self.Mat_K[2, 2] = 1.0
        self.Mat_K[0, 2] = c_x
        self.Mat_K[1, 2] = c_y

    def setMatrix_R(self, poseMat):

        for i in range(len(poseMat)):
            self.Mat_R.append(poseMat[i][:,0:3])
        # self.Mat_R[0,0]= math.cos(fRoll) * math.cos(fYaw) - math.sin(fRoll) * math.sin(fPitch) * math.sin(fYaw)
        # self.Mat_R[0, 1] = -math.sin(fRoll) * math.cos(fPitch)
        # self.Mat_R[0, 2] = (math.cos(fRoll) * math.sin(fYaw)) + (math.sin(fRoll) * math.sin(fPitch) * math.cos(fYaw))
        # self.Mat_R[1, 0] = (math.sin(fRoll) * math.cos(fYaw)) + (math.cos(fRoll) * math.sin(fPitch) * math.sin(fYaw))
        # self.Mat_R[1, 1] = math.cos(fRoll) * math.cos(fPitch)
        # self.Mat_R[1, 2] = (math.sin(fRoll) * math.sin(fYaw)) - (math.cos(fRoll) * math.sin(fPitch) * math.cos(fYaw))
        # self.Mat_R[2, 0] = -math.cos(fPitch) * math.sin(fYaw)
        # self.Mat_R[2, 1] = math.sin(fPitch)
        # self.Mat_R[2, 2] = math.cos(fPitch) * math.cos(fYaw)


    def setMatrix_T(self,poseMat,fTy,fScale_X,fScale_Z):
        for i in range(len(poseMat)):
            temp_T=np.zeros((3, 1))
            temp_T[0] = fScale_X*poseMat[i][0,3]
            temp_T[1] = fTy
            temp_T[2] = fScale_X*poseMat[i][2,3]

            self.Mat_T.append(temp_T)
        #print(self.Mat_T)


    def calMatrix_P(self):
        for i in range(len(self.Mat_T)):
            temp_T=-np.dot(self.Mat_R[i],self.Mat_T[i])
            temp_P=np.dot(self.Mat_K,np.concatenate((self.Mat_R[i],temp_T),axis=1))
            self.Mat_P.append(temp_P)

    def getFrmSize(self):
        return self.img_width

    def initCamMdl(self, sParamRng = ParamRng()):
        
        secretsGenerator = secrets.SystemRandom()
        self.Ty = secretsGenerator.uniform(sParamRng.fTyMin, sParamRng.fTyMax)
        self.Scale_X = secretsGenerator.uniform(sParamRng.fScaleMin_X, sParamRng.fScaleMax_X)
        self.Scale_Z = secretsGenerator.uniform(sParamRng.fScaleMin_Z, sParamRng.fScaleMax_Z)
        
       
        
    def setReprojErr(self,fReprojErr):
        self.fReprojErr = fReprojErr
