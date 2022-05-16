import os
from PIL import Image
import numpy as np
import cv2
import src.detect.VehicleLampDet as VehicleLampDet

#convert detection labels to annotation labels(can be user-defined)
labels_detect2anno={'person':'Pedestrian', 'bicycle':'Bicycle', 'car':'Car','pedestrian':'Pedestrian', 
                    'motorcycle':'Motorcycle', 'bus':'Bus', 'truck':'Truck','Tricycle':'Tricycle'}
DET_MIN_SIZE = 24
class Detecter(object):
    def __init__(self):
        self.detect_model = None   
 
    #https://stackoverflow.com/questions/40690598/can-keras-with-tensorflow-backend-be-forced-to-use-cpu-or-gpu-at-will
    #todo, cpu and multiple-gpu cases
    def load_model(self, model_name, view_name, anno_label_list, det_config): 
        self.model_name = model_name
        self.view_name = view_name
        self.det_lamp = det_config['DO_LAMP_DETECTION']
        self.device = det_config['DEVICE']
        self.nms_ratio = det_config['NMS_RATIO']
        self.anno_label_list = anno_label_list

        if 'CPU' in self.device:
            gpu_num = 0 
        else: 
            gpu_num = 1
        if self.model_name == "Mask RCNN": 
            self.min_det_score = det_config['MASK_RCNN_SCORE_THRESHOLD']
            self.log_dir = det_config['MASK_RCNN_LOG_DIR'] #not used
            if self.view_name == "Survelliance": 
                self.load_classes(det_config['CLASSES_PATH'])#support coco, c2
                self.model_path = det_config['MASK_RCNN_MODEL_PATH']
            else: #view_name == 'Ego-view'
                self.load_classes(det_config['CLASSES_PATH_EGO'])#support coco, c2
                self.model_path = det_config['MASK_RCNN_EGO_MODEL_PATH']
            import src.detect.mask_rcnn.model_v2 as maskrcnn_model
            from src.detect.mask_rcnn.config_v2 import Config
            class InferenceConfig(Config):
                    GPU_COUNT = 1 #seems not working when set GPU=0 for cpu 
                    IMAGES_PER_GPU = 1
                    DETECTION_MIN_CONFIDENCE = det_config['MASK_RCNN_SCORE_THRESHOLD']#0.5
            config = InferenceConfig()
            config.NUM_CLASSES = self.det_num_classes + 1
            config.IMAGE_META_SIZE = 1 + 3 + 3 + 4 + 1 + config.NUM_CLASSES
            config.NAME = self.det_data_name
            self.detect_model = maskrcnn_model.MaskRCNN(
                                    mode="inference", 
                                    model_dir=self.log_dir,# det_config['MASK_RCNN_MODEL_DIR'], 
                                    config=config)
            self.detect_model.load_weights(self.model_path, by_name=True)#det_config['MASK_RCNN_MODEL_PATH'], by_name=True)
        elif self.model_name =='YOLO v4': #only coco data is supported 
            self.min_det_score = det_config['YOLO_SCORE_THRESHOLD']
            from src.detect.yolo4.yolo import YOLO
                #print("Loading yolo: %s\n"%YOLO_MODEL_PATH)
            self.detect_model = YOLO(model_path=det_config['YOLO_MODEL_PATH'], \
                             anchors_path=det_config['YOLO_ANCHORS_PATH'],\
                             classes_path=det_config['YOLO_CLASSES_PATH'], \
                             gpu_num=gpu_num)
            self.load_classes(det_config['YOLO_CLASSES_PATH'])
    def load_classes(self, class_filename): 
        labels2names_coco = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}#, 9: 'boat'}#, 10: 'trafficlight', 11: 'firehydrant', 12: 'stopsign', 13: 'parkingmeter'}
        labels2names_c2 = {0:'car',1:'person'}
        labels2name_apollo = {0:'person',1:'rider',2:'car', 3:'motor', 4:'bicycle', 5:'truck',6:'bus',7:'tricycle'}
        purename = os.path.basename(class_filename)
        if 'coco' in purename: 
            self.det_num_classes = 80
            self.det_data_name = 'coco'
            self.labels2names = labels2names_coco
        elif 'c2' in purename:#2type
            self.det_num_classes = 2
            self.det_data_name = 'c2'
            self.labels2names = labels2names_c2            
        elif 'apollo' in purename: 
            self.det_num_classes = 8
            self.det_data_name = 'apollo'
            self.labels2names = labels2name_apollo
        else: 
            print("unsupported type")
    #boxes[x1,y1,x2,y2]    
    def nms(self, boxes, max_bbox_overlap, scores=None):
        if len(boxes) == 0:
            return []

        boxes = boxes.astype(np.float)
        pick = []

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2] 
        y2 = boxes[:, 3] 

        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        if scores is not None:
            idxs = np.argsort(scores)
        else:
            idxs = np.argsort(y2)

        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(i)

            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)

            overlap = (w * h) / area[idxs[:last]]

            idxs = np.delete(
                idxs, np.concatenate(
                    ([last], np.where(overlap > max_bbox_overlap)[0])))

        return pick

    def detect_raw(self, bgr_array):
        rgb_array = cv2.cvtColor(bgr_array, cv2.COLOR_BGR2RGB)
        if self.detect_model is None: 
            print("Failed to load detection model")
            return
        if self.model_name =="Mask RCNN": 
            raw_results = self.detect_model.detect([rgb_array], verbose=0)
            r = raw_results[0]
            boxes, labels, scores = r['rois'], r['class_ids']-1, r['scores'] #class_id -1, reove background
        elif self.model_name == 'YOLO v4': 
            img_raw_bak = rgb_array.copy()
            img_raw_pil = Image.fromarray(img_raw_bak)
            boxes, labels, scores = self.detect_model.detect_image(img_raw_pil)
        if len(boxes)<1: 
            return boxes, labels, scores
        else: 
            new_bboxes = boxes[:,[1,0,3,2]]
            picks = self.nms(new_bboxes, self.nms_ratio, scores)
            return new_bboxes[picks[:],:], labels[picks[:]], scores[picks[:]]

    #return detect format: [lobject_id(fake),label_id, x1, y1, x2,y2, score, source]
    def detect(self, bgr_array): 
        boxes, labels, scores = self.detect_raw(bgr_array)
        img_w, img_h = bgr_array.shape[1], bgr_array.shape[0]
        valid_dets = []
        num_det = boxes.shape[0]
        centers = []

        for idx in range(num_det): 
            if scores[idx]< self.min_det_score: 
                continue
            lbl_key = labels[idx]#+1
            if not lbl_key in self.labels2names.keys(): 
                continue
            label_str = self.labels2names[lbl_key]
            if not  label_str  in labels_detect2anno:
                continue 
            anno_label_id = self.anno_label_list.index(labels_detect2anno[label_str])
            #new unified labels
            left = max(0, min(img_w-1, boxes[idx,0]))
            top =  max(0, min(img_h-1, boxes[idx,1]))
            right = max(0, min(img_w-1, boxes[idx,2]))
            bottom = max(0, min(img_h-1, boxes[idx,3]))
            
            if not anno_label_id ==0: 
                if (bottom-top)<DET_MIN_SIZE or (right-left)<DET_MIN_SIZE: #ignore tiny object
                    continue 
            else: #person usually have smaller width, so just conisder height
                if(bottom-top)<DET_MIN_SIZE: 
                    continue
            
            out_det_vec = (anno_label_id,
                           left, top, right, bottom, 
                          scores[idx])
            valid_dets.append(out_det_vec)
            if self.det_lamp:
                temp_ROI = bgr_array[top:bottom, left:right,:]
                temp_ROI.astype('uint8')
                # print(temp_ROI.type)
                hsv = cv2.cvtColor(temp_ROI, cv2.COLOR_BGR2HSV)
                centos = VehicleLampDet.Detect(hsv)
                centos = [int(centos[0] + boxes[idx, 1]), int(centos[1] + boxes[idx,0])]
                centers.append(centos) #return lamp centers
            else: 
                centers.append([int((left+right)/2), int(top+bottom)/2])#return box centers   
        valid_dets = self.post_suppress(valid_dets)
        '''
        import copy
        canvas = copy.deepcopy(bgr_array)
        for box in valid_dets: 
            cv2.rectangle(canvas, (box[1], box[2]), (box[3], box[4]), (0,0,255), 2)
        saver = 'debug/{}.jpg'.format(self.g_index)
        cv2.imwrite(saver, canvas)
        self.g_index += 1
        #cv2.imshow("detect", canvas)
       # cv2.waitKey(10)
       '''
    
        return valid_dets, centers
    
    #processing case: small bbox in a large bbox->remove large bb
    def post_suppress(self, valid_dets, max_bbox_overlap=0.9): 
        if len(valid_dets) ==0: 
            return []
        boxes = np.asarray(valid_dets)[:,1:5]
        pick = []
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2] 
        y2 = boxes[:, 3] 

        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(area)#ascending 
        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(i)
            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])

            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)

            overlap = (w * h) / area[i]
            idxs = np.delete(
                idxs, np.concatenate(
                    ([last], np.where(overlap > max_bbox_overlap)[0])))
        out_dets = []
        for i in pick: 
            out_dets.append(valid_dets[i])
        return out_dets




