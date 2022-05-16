from .simple_tracker import Tracker as SimpleTracker
from .JRETrack.multitracker import JDETracker
from .kcf.kcftracker import KCFTracker
import numpy as np

class TRACKER(object): 
    def __init__(self, track_config): 
        self.track_alg = track_config['TRACK_MODE']
        self.device = track_config['DEVICE']
        self.reid_modelname = track_config['REID_MODEL']
        self.load_model()
        self.tracks = []
        self.frame_num = 0
    
    def load_model(self): 

        if self.track_alg =="SIMPLE": 
            self.tracker = SimpleTracker(60, 30, 300, 100)
        elif self.track_alg =="DEEP_SORT": 
            #max_distance = 0.5
            #nn_budget = 100
            #metric = NearestNeighborDistanceMetric("euclidean", max_distance, nn_budget)
            #self.tracker = DeepSortTracker(metric, reid_model_filename=self.reid_modelname, device=self.device)
            #self.tracker = DeepSortTracker(self.reid_modelname)
            self.tracker = JDETracker(self.reid_modelname)
        elif self.track_alg =="KCF": 
            self.tracker = KCFTracker(True, True, False) ## hog, fixed_window, multiscale
        else:  
            print("wrong tracking algorithm")
            return 
    
    def reset(self): 
        try:
            if self.track_alg == "DEEP_SORT": 
                self.tracker.release() #release gpu memory first 
            del self.tracker
            self.load_model()
        except NameError: #not defined load
            self.load_model()         
        #self.tracker.reset()
        self.frame_num =0

    #input detection: each row: [label_id, left, top, right, bottom, confidence]
    #curr_result, history(curr_result): [frame_idx, track_id, label_id, detection_index, x1,y1,x2,y2 ]
    def update(self, detections, bgr_image, frame_idx, force_matching=False): 
        tlbrs = []
        centers = []
        confidence = []
        labels = []
        for det in detections:
            if len(det)<=0: 
                continue 
            left, top,right,bottom = det[1], det[2], det[3], det[4]
            labels.append(int(det[0]))
            tlbrs.append(np.array([int(left), int(top), int(right), int(bottom)]))
            centers.append(np.array([(left + right) / 2, (top + bottom) /2]))
            confidence.append(det[5])
       # if not boxes: 
       #     return #empty box #need to track on no-detection cases
        if self.track_alg == "SIMPLE": 
            self.tracker.update(centers)
        elif self.track_alg == "DEEP_SORT":
            self.tracker.update(bgr_image, np.array(tlbrs), confidence, labels)
        elif self.track_alg == "KCF": 
            if(self.frame_num ==0): #KCF only works in single object case
                x,y,w,h = xywhs[0][0], tlbrs[0][1], xywhs[0][2]-xywhs[0][0], xywhs[0][3]-xywhs[0][1]
                self.tracker.init([x,y,w,h], bgr_image,labels[0])
                self.frame_num = 0
            else: 
                self.tracker.update(bgr_image)
                
    def get_current_result(self):
        curr_result = [] 
        for tr in self.tracker.get_tracklets():  
            curr_result.append(tr)
        return curr_result




