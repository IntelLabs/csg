
import random
from .CamCal_config import Config
import numpy as np
import math
from .ParamRng import ParamRng
import secrets

class CamParam:
    def __init__(self):
        self.R_Mat = []
        self.Fx = 0
        self.Fy = 0
        self.Cx = 0
        self.Cy = 0
        self.Pitch = 0
        self.Yaw = 0
        self.Roll = 0
        self.Tx = 0
        self.Ty = 0
        self.Tz = 0
        self.config = Config()
        self.img_width =1920
        self.img_height =1080
        self.Mat_K = np.zeros((3, 3))
        self.Mat_R = np.zeros((3, 3))
        self.Mat_T = np.zeros((3, 1))
        self.Mat_P = np.zeros((3, 4))
        self.fReprojErr=0

    def setMatrix_K(self,f_x, f_y, c_x, c_y):
        self.Mat_K[0, 0] = f_x
        self.Mat_K[1, 1] = f_y
        self.Mat_K[2, 2] = 1.0
        self.Mat_K[0, 2] = c_x
        self.Mat_K[1, 2] = c_y

    def setMatrix_R(self, fRoll, fPitch, fYaw):

        if Config.COORD_SYS_TYP == 0:
            self.Mat_R[0,0]= math.cos(fRoll) * math.cos(fYaw) - math.sin(fRoll) * math.sin(fPitch) * math.sin(fYaw)
            self.Mat_R[0, 1] = -math.sin(fRoll) * math.cos(fPitch)
            self.Mat_R[0, 2] = (math.cos(fRoll) * math.sin(fYaw)) + (math.sin(fRoll) * math.sin(fPitch) * math.cos(fYaw))
            self.Mat_R[1, 0] = (math.sin(fRoll) * math.cos(fYaw)) + (math.cos(fRoll) * math.sin(fPitch) * math.sin(fYaw))
            self.Mat_R[1, 1] = math.cos(fRoll) * math.cos(fPitch)
            self.Mat_R[1, 2] = (math.sin(fRoll) * math.sin(fYaw)) - (math.cos(fRoll) * math.sin(fPitch) * math.cos(fYaw))
            self.Mat_R[2, 0] = -math.cos(fPitch) * math.sin(fYaw)
            self.Mat_R[2, 1] = math.sin(fPitch)
            self.Mat_R[2, 2] = math.cos(fPitch) * math.cos(fYaw)
        elif Config.COORD_SYS_TYP == 1:
            self.Mat_R[0, 0] = (-math.cos(fRoll) * math.sin(fYaw)) - (math.sin(fRoll) * math.sin(fPitch) * math.cos(fYaw))
            self.Mat_R[0, 1] = (-math.cos(fRoll) * math.cos(fYaw)) - (math.sin(fRoll) * math.sin(fPitch) * math.cos(fYaw))
            self.Mat_R[0, 2] = math.sin(fRoll) * math.cos(fPitch)
            self.Mat_R[1, 0] = (-math.sin(fRoll) * math.sin(fYaw)) + (math.cos(fRoll) * math.sin(fPitch) * math.cos(fYaw))
            self.Mat_R[1, 1] = (-math.sin(fRoll) * math.cos(fYaw)) - (math.cos(fRoll) * math.sin(fPitch) * math.sin(fYaw))
            self.Mat_R[1, 2] = -math.cos(fRoll) * math.cos(fPitch)
            self.Mat_R[2, 0] = math.cos(fPitch) * math.cos(fYaw)
            self.Mat_R[2, 1] = -math.cos(fPitch) * math.sin(fYaw)
            self.Mat_R[2, 2] = math.sin(fPitch)

    def setMatrix_T(self,Tx, Ty, Tz):

        self.Mat_T[0] = Tx
        self.Mat_T[1] = Ty
        self.Mat_T[2] = Tz


    def calMatrix_P(self):
        temp_T=-np.dot(self.Mat_R,self.Mat_T)
        self.Mat_P=np.dot(self.Mat_K,np.concatenate((self.Mat_R,temp_T),axis=1))

    def getFrmSize(self):
        return self.img_width

    def initCamMdl(self, sParamRng = ParamRng()):
        secretsGenerator = secrets.SystemRandom()
        
        self.Fx = secretsGenerator.uniform(sParamRng.fFxMin, sParamRng.fFxMax)
        self.Fy = secretsGenerator.uniform(sParamRng.fFyMin, sParamRng.fFyMax)
        self.Cx = secretsGenerator.uniform(sParamRng.fCxMin, sParamRng.fCxMax)
        self.Cy = secretsGenerator.uniform(sParamRng.fCyMin, sParamRng.fCyMax)
        self.Roll = secretsGenerator.uniform(sParamRng.fRollMin, sParamRng.fRollMax)
        self.Pitch = secretsGenerator.uniform(sParamRng.fPitchMin, sParamRng.fPitchMax)
        self.Yaw = secretsGenerator.uniform(sParamRng.fYawMin, sParamRng.fYawMax)
        self.Tx = secretsGenerator.uniform(sParamRng.fTxMin, sParamRng.fTxMax)
        self.Ty = secretsGenerator.uniform(sParamRng.fTyMin, sParamRng.fTyMax)
        self.Tz = secretsGenerator.uniform(sParamRng.fTzMin, sParamRng.fTzMax)
        
        #self.Fx = random.uniform(sParamRng.fFxMin, sParamRng.fFxMax)
        #self.Fy = random.uniform(sParamRng.fFyMin, sParamRng.fFyMax)
        #self.Cx = random.uniform(sParamRng.fCxMin, sParamRng.fCxMax)
        #self.Cy = random.uniform(sParamRng.fCyMin, sParamRng.fCyMax)
        #self.Roll = random.uniform(sParamRng.fRollMin, sParamRng.fRollMax)
        #self.Pitch = random.uniform(sParamRng.fPitchMin, sParamRng.fPitchMax)
        #self.Yaw = random.uniform(sParamRng.fYawMin, sParamRng.fYawMax)
        #self.Tx = random.uniform(sParamRng.fTxMin, sParamRng.fTxMax)
        #self.Ty = random.uniform(sParamRng.fTyMin, sParamRng.fTyMax)
        #self.Tz = random.uniform(sParamRng.fTzMin, sParamRng.fTzMax)

    def setReprojErr(self,fReprojErr):
        self.fReprojErr = fReprojErr
