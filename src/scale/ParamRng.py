
from .CamCal_config import Config
class ParamRng:
    def __init__(self):
        # self.fFxMax = 0
        # self.fFxMin = 0
        # self.fFyMax = 0
        # self.fFyMin = 0
        # self.fCxMax = 0
        # self.fCxMin = 0
        # self.fCyMax = 0
        # self.fCyMin = 0
        # self.fRollMax = 0
        # self.fRollMin = 0
        # self.fPitchMax = 0
        # self.fPitchMin = 0
        # self.fYawMax = 0
        # self.fYawMin = 0
        self.fTyMax = Config.CalCamHeiMax
        self.fTyMin = Config.CalCamHeiMin
        self.fScaleMax_X= Config.ScaleMax_X
        self.fScaleMin_X = Config.ScaleMin_X
        self.fScaleMax_Z= Config.ScaleMax_Z
        self.fScaleMin_Z = Config.ScaleMin_Z

