import os

ROS_PATH = '/opt/ros/kinetic/lib/python2.7/dist-packages'
CURR_PATH = os.path.dirname(__file__)

SOURCE = {'Auto':1, 'Track':2, 'Manual':3} 

DETECT_MODELS = ('Mask RCNN', "YOLO")

DET_CONFIG = dict()
DET_CONFIG['DO_LAMP_DETECTION'] = False

DET_CONFIG['MASK_RCNN_LOG_DIR'] = os.path.join(CURR_PATH, 'data', 'mask_rcnn', 'logs')
DET_CONFIG['CLASSES_PATH'] = os.path.join(CURR_PATH, 'data', 'coco_classes.txt')
DET_CONFIG['MASK_RCNN_MODEL_PATH'] = os.path.join(CURR_PATH, 'data', 'mask_rcnn', 'mask_rcnn_coco.h5')
DET_CONFIG['MASK_RCNN_EGO_MODEL_PATH'] = os.path.join(CURR_PATH, 'data', 'mask_rcnn_ego', 'mask_rcnn_apollo.h5')
DET_CONFIG['CLASSES_PATH_EGO'] = os.path.join(CURR_PATH, 'data', 'apollo_classes.txt')

DET_CONFIG['MASK_RCNN_SCORE_THRESHOLD'] = 0.7

DET_CONFIG['YOLO_MODEL_PATH'] = os.path.join(CURR_PATH, 'data','yolo', 'yolo4_weights.h5')
DET_CONFIG['YOLO_ANCHORS_PATH'] = os.path.join(CURR_PATH, 'data','yolo', 'yolo4_anchors.txt')
DET_CONFIG['YOLO_CLASSES_PATH'] = os.path.join(CURR_PATH, 'data', 'coco_classes.txt')
DET_CONFIG['YOLO_SCORE_THRESHOLD'] = 0.6

DET_CONFIG['DEVICE'] = '/GPU:0'#/CPU:0'
DET_CONFIG['NMS_RATIO'] = 0.8

#Track 
TRACK_CONFIG = dict()
TRACK_CONFIG['TRACK_MODE'] = 'DEEP_SORT' 
TRACK_CONFIG['REID_MODEL'] = os.path.join(CURR_PATH, 'data', 'vehicle_reid', 'vehicle_reid_model.pb')#used by deep_sort
TRACK_CONFIG['DEVICE'] = '/CPU:0'

#Lane
LANE_CONFIG = dict()
LANE_CONFIG['MODEL'] = os.path.join(CURR_PATH, 'data', 'lane', 'model_culane.pkl')
LANE_CONFIG['DEVICE'] = 'cuda:0'

#depth
DEPTH_CONFIG = dict()
DEPTH_CONFIG['MODEL'] = os.path.join(CURR_PATH, 'data', 'depth')
DEPTH_CONFIG['DEVICE'] = 'cuda:0'
