import numpy as np
import torch
import cv2
import time
import copy
import gc
import itertools
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import MinMaxScaler,StandardScaler
from scipy.optimize import curve_fit
from scipy.optimize import linear_sum_assignment as linear_assignment
import matplotlib.pyplot as plt

#import sys
#sys.path.append('/home/lidanzha/hdd/work/CSG/critical-scenario-generation/')


from src.road.PINet.parameters import Parameters as Lane_Param
from src.road.PINet.agent import Agent
import src.road.PINet.util as Lane_Util
from src.convert_util import project_2d_to_3d_depth, project_2d_to_3d_depth_arr
from src.convert_util import get_abs_pose, project_3d_to_2d


class LaneDetect(object): 
    def __init__(self, model_path, device='cuda:0'):#cpu'): 
        self.p = Lane_Param()
        self.lane_agent = Agent()
        self.lane_agent.load_weight_file(model_path, device)
        if 'cuda' in device: 
            self.lane_agent.cuda()
        self.lane_agent.evaluate_mode()
        self.device = device

    def detect_image(self, bgr): 
        ori_height, ori_width = bgr.shape[0], bgr.shape[1]
        frame = copy.deepcopy(bgr)
        if 'cuda' in self.device: 
            torch.cuda.synchronize()
        frame = cv2.resize(frame, (512,256))/255.0
        dst_height, dst_width= frame.shape[0], frame.shape[1]
        frame = np.rollaxis(frame, axis=2, start=0)
        prevTime = time.time()
        px, py, ti = self.test(np.array([frame])) 
        curTime = time.time()
        sec = curTime - prevTime
        fps = 1/(sec)
        ti[0] = cv2.resize(ti[0], (ori_width,ori_height))
        ori_px = [ [ [int(ori_width/dst_width*x) for x in pppx] for pppx in ppx ] for ppx in px ]
        ori_py = [ [ [int(ori_height/dst_height*y) for y in pppy] for pppy in ppy ] for ppy in py ]
        if 0:
            cv2.putText(ti[0], str(fps), (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
            cv2.imshow('frame',ti[0])
            cv2.waitKey(-1)
        return ori_px, ori_py, ti[0]
    
    def poly_fit(self, px, py): 
        poly_coeffs = []
        for i in range(len(px)): 
            xs = px[i][0]
            ys = py[i][0]
            coeffs = np.polyfit(ys, xs, 2)
            poly_coeffs.append(coeffs)

    def test(self, test_images, thresh = 0.96, index= -1):
        with torch.no_grad(): 
            result = self.lane_agent.predict_lanes_test(test_images)
        confidences, offsets, instances = result[index]
        num_batch = len(test_images)

        out_x = []
        out_y = []
        out_images = []
        for i in range(num_batch):
            # test on test data set
            image = copy.deepcopy(test_images[i])
            image =  np.rollaxis(image, axis=2, start=0)
            image =  np.rollaxis(image, axis=2, start=0)*255.0
            image = image.astype(np.uint8).copy()

            confidence = confidences[i].view(self.p.grid_y, self.p.grid_x).cpu().data.numpy()

            offset = offsets[i].cpu().data.numpy()
            offset = np.rollaxis(offset, axis=2, start=0)
            offset = np.rollaxis(offset, axis=2, start=0)
            
            instance = instances[i].cpu().data.numpy()
            instance = np.rollaxis(instance, axis=2, start=0)
            instance = np.rollaxis(instance, axis=2, start=0)

            # generate point and cluster
            raw_x, raw_y = self.generate_result(confidence, offset, instance, thresh)

            # eliminate fewer points
            in_x, in_y = self.eliminate_fewer_points(raw_x, raw_y)
                    
            # sort points along y 
            in_x, in_y = Lane_Util.sort_along_y(in_x, in_y)  

            result_image = Lane_Util.draw_points(in_x, in_y, copy.deepcopy(image))

            out_x.append(in_x)
            out_y.append(in_y)
            out_images.append(result_image)

        return out_x, out_y,  out_images

    def generate_result(self, confidance, offsets,instance, thresh):
    
        mask = confidance > thresh

        grid = self.p.grid_location[mask]
        offset = offsets[mask]
        feature = instance[mask]

        lane_feature = []
        x = []
        y = []
        for i in range(len(grid)):
            if (np.sum(feature[i]**2))>=0:
                point_x = int((offset[i][0]+grid[i][0])*self.p.resize_ratio)
                point_y = int((offset[i][1]+grid[i][1])*self.p.resize_ratio)
                if point_x > self.p.x_size or point_x < 0 or point_y > self.p.y_size or point_y < 0:
                    continue
                if len(lane_feature) == 0:
                    lane_feature.append(feature[i])
                    x.append([point_x])
                    y.append([point_y])
                else:
                    flag = 0
                    index = 0
                    min_feature_index = -1
                    min_feature_dis = 10000
                    for feature_idx, j in enumerate(lane_feature):
                        dis = np.linalg.norm((feature[i] - j)**2)
                        if min_feature_dis > dis:
                            min_feature_dis = dis
                            min_feature_index = feature_idx
                    if min_feature_dis <= self.p.threshold_instance:
                        lane_feature[min_feature_index] = (lane_feature[min_feature_index]*len(x[min_feature_index]) + feature[i])/(len(x[min_feature_index])+1)
                        x[min_feature_index].append(point_x)
                        y[min_feature_index].append(point_y)
                    elif len(lane_feature) < 12:
                        lane_feature.append(feature[i])
                        x.append([point_x])
                        y.append([point_y])
                    
        return x, y

    def eliminate_fewer_points(self, x, y):
        # eliminate fewer points
        out_x = []
        out_y = []
        for i, j in zip(x, y):
            if len(i)>5:
                out_x.append(i)
                out_y.append(j)     
        return out_x, out_y  

    def release_from_gpu(self): 
        del self.lane_agent
        gc.collect()
        torch.cuda.empty_cache()

def avg_two_line(pt1, pt2, weight=0.6): 
    coeff_lane1 = np.polyfit(pt1[:,0], pt1[:,1], 2)
    poly = np.poly1d(coeff_lane1)
    num_pt = len(pt2)
    new_pt2 = np.zeros((num_pt, 2))
    for i in range(num_pt): 
        x = pt2[i][0] 
        y1 = poly(x)
        y2 = pt2[i][1]
        new_pt2[i,0] = x
        new_pt2[i,1] = weight*y1 + (1-weight)*y2 
    return new_pt2
       
def inRect(box, pts): 
    x1,y1,x2,y2 = min(box[:,0]),min(box[:,1]), max(box[:,0]), max(box[:,1])
    valid  =[]
    for i in range(len(pts)): 
        valid.append(((pts[i,0]>x1) and (pts[i,1]>y1) and (pts[i,0]<x2) and (pts[i,1]<y2)))
      #      valid.append(True)
      #  else: 
      #      valid.append(False)
    return valid

def sliding_window(data, window_size = 10, overlap_size = 3):
    data_len = len(data)
    out = []
    for i in range(0, data_len-window_size, window_size-overlap_size): 
        out.append(list(range(i, i+window_size)))#data[i:i+window_size])
    #for rest frames
    out.append(list(range(data_len-window_size, data_len)))#data[data_len-window_size:])
    return out 

def fit_surface(x,y,z, order=3): 
    ncols = (order+1)**2
    G = np.zeros((x.size, ncols))
    ij = itertools.product(range(order+1), range(order+1))
    for k, (i,j) in enumerate(ij):
        G[:,k] = x**i * y**j
    m, _, _, _ = np.linalg.lstsq(G, z, rcond=None)
    return m
def eval_surface(x, y, m):
    order = int(np.sqrt(len(m))) - 1
    ij = itertools.product(range(order+1), range(order+1))
    z = np.zeros_like(x)
    for a, (i,j) in zip(m, ij):
        z += a * x**i * y**j
    return z

def cluster3d(raw_points3d, eps=0.05, min_samples=10, ratio=0.1):#2): 
    #input: raw_points3d: array of points(3xN)

    raw_points3d = np.transpose(raw_points3d)
    points3d_xy = raw_points3d[:2,:]
    points3d_xy = np.transpose(points3d_xy)

    scaler = MinMaxScaler()
    lines = scaler.fit_transform(points3d_xy)
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(lines) 
    labels = db.labels_
    lines = scaler.inverse_transform(lines) #getting back our original values

    num_clusters = np.max(labels) + 1
    if 0: 
        for label_id in np.unique(labels): 
            idx = np.where(labels==label_id)
            plt.scatter(lines[idx, 0], lines[idx,1],  label=str(label_id), cmap="jet")#, alpha=0.1) #c=labels[idx],
        plt.legend()
        plt.show()
    #remove too short segments 
    unq_lbl, count_lbl = np.unique(labels, return_counts=True)
    min_count_num = ratio * max(count_lbl)

    #fit lines and create new fake points by interpolation 
    #surface_coeff = fit_surface(raw_points3d[:,0],raw_points3d[:,1],raw_points3d[:,2])#searching for z 
    lanes3d = dict()
    lanes3d_coeff = dict()
    lane_count = 0 #
    for i in range(num_clusters): 
        idx = np.where(labels==i)
        if len(idx[0])< min_count_num: 
            continue 
        xs = lines[idx[0],0]
        ys = lines[idx[0],1]
        xs_range = max(xs)-min(xs)
        ys_range = max(ys)-min(ys)
        z_mean = raw_points3d[2, idx[0]].mean()
        if xs_range >= ys_range: 
            coeff = np.polyfit(xs, ys, 2)
            ixs = np.linspace(min(xs), max(xs), 100)
            poly = np.poly1d(coeff)
            iys = poly(ixs)
        else: 
            coeff = np.polyfit(ys, xs, 2)
            iys = np.linspace(min(ys), max(ys), 100)
            poly = np.poly1d(coeff)
            ixs = poly(iys)
        izs = np.ones((ixs.shape[0], 1))*z_mean#eval_surface(ixs, iys, surface_coeff)
        lanes3d[str(lane_count)] = np.hstack((np.expand_dims(ixs,1), 
                                             np.expand_dims(iys,1),
                                             izs))
        lane_count += 1

    return lanes3d
def reproject3d_to_frames(lanes_points3d, image_size,frame_list, 
                          intrisic, extrisic_list, depth_list,
                          point_num=5): 
    num_frames =len(frame_list) 
    image_width, image_height = image_size[0], image_size[1]
    image_bound = [[0,0], [image_width-1, 0], 
                       [image_width-1, image_height-1], [0, image_height-1]] 
    lanes2d = dict()             
    for i in range(num_frames): 
        fid = frame_list[i]
        pose = extrisic_list[i]
        depth = depth_list[i]
        bev_rect = []
        bev_min_x, bev_min_y, bev_max_x, bev_max_y =  100000, 10000, -10000, -10000
        for pt in image_bound: 
            temp = project_2d_to_3d_depth(np.asarray(pt), 
                                          intrisic, pose,
                                          depth[int(pt[1]), int(pt[0])])
            bev_min_x = min(bev_min_x, temp[0])
            bev_min_y = min(bev_min_y, temp[1]) 
            bev_max_x = max(bev_max_x, temp[0])
            bev_max_y = max(bev_max_y, temp[1])
            bev_rect.append(temp)
        #for each lane, get project on current frame
        curr_lanes = dict()
        for lane_id, pt3d in lanes_points3d.items(): 
            min_x,min_y,max_x,max_y = min(pt3d[:,0]), min(pt3d[:,1]),max(pt3d[:,0]), max(pt3d[:,1])
            #surface_coeff = fit_surface(pt3d[:,0],pt3d[:,1],pt3d[:,2])
            #assume: flat plane with same height 
            z_mean = pt3d[:,2].mean()
            if (max_x-min_x) >= (max_y-min_y): 
                coeff = np.polyfit(pt3d[:,0], pt3d[:,1], 2)
                ixs = np.linspace(bev_min_x, bev_max_x, point_num*2)
                poly = np.poly1d(coeff)
                iys = poly(ixs)
            else: 
                coeff = np.polyfit(pt3d[:,1], pt3d[:,0], 2)
                iys = np.linspace(bev_min_y, bev_max_y, point_num*2)
                poly = np.poly1d(coeff)
                ixs = poly(iys)
            izs = np.ones((ixs.shape[0], 1))*z_mean
            ipt3d = np.hstack((np.expand_dims(ixs,1), 
                               np.expand_dims(iys,1),
                               izs))      
            ipt2d = project_3d_to_2d(np.transpose(ipt3d), intrisic, pose)
            ipt2d = np.transpose(ipt2d)
            valid = inRect(np.array(image_bound), ipt2d)
            valid_pt2d = list(itertools.compress(ipt2d, valid))
            if len(valid_pt2d) ==0: 
                curr_lanes[lane_id] = np.array([])
            else:
                #resample. 
                valid_pt2d = np.array(valid_pt2d)
                coeff = np.polyfit(valid_pt2d[:,0], valid_pt2d[:,1], 2)
                ixs = np.linspace(min(valid_pt2d[:,0]), max(valid_pt2d[:,0]), point_num)
                poly = np.poly1d(coeff)
                iys = poly(ixs)
                curr_lanes[lane_id] = np.hstack((np.expand_dims(ixs,1), 
                                                np.expand_dims(iys,1))) 
    
        lanes2d[str(fid)] = curr_lanes
    return lanes2d

def calculate_lane_distance(line1_pt, line2_pt): 
    #project line1 to line2
    coeff_lane1 = np.polyfit(line1_pt[:,0], line1_pt[:,1], 2)
    poly = np.poly1d(coeff_lane1)
    #min_x_l1, min_y_l1, max_x_l1, max_y_l1 = min(line1_pt[:,0]),min(line1_pt[:,1]),\
    #                                         max(line1_pt[:,0]),max(line1_pt[:,1])
    #min_x_l2, min_y_l2, max_x_l2, max_y_l2 = min(line2_pt[:,0]),min(line2_pt[:,1]),\
    #                                         max(line2_pt[:,0]),max(line2_pt[:,1])    
    #only consider bev , no z 
    dists = []
    for pt in line2_pt: 
        #get its point in line1 
        iys = poly(pt[0])
        dists.append(abs(iys-pt[1]))
    return np.array(dists).mean()

def track_lanes(new_lane3d, old_lanes3d, dist_thresh=10.):
    num_new_lanes = len(new_lane3d)
    num_old_lanes = len(old_lanes3d)
    new_laneids = list(new_lane3d.keys())
    old_laneids = list(old_lanes3d.keys())
    max_lane_id = max(map(int, old_laneids))
    
    if 0: 
        colors = []
        for k,v in new_lane3d.items(): 
            plt.scatter(v[:,0], v[:,1],label='new'+k)
        for k,v in old_lanes3d.items(): 
            plt.scatter(v[:,0], v[:,1], label='old'+k)
        plt.legend(loc='best')
        plt.show()
    dist_mat = np.zeros((num_new_lanes, num_old_lanes))
    for i, (lane_id_new, pt_new) in enumerate(new_lane3d.items()): 
        for j, (lane_id_old, pt_old) in enumerate(old_lanes3d.items()):
            dist_mat[i,j] = calculate_lane_distance(pt_new, pt_old)
    
    #associate, matches, unmatched_new_line, unmatched_old_lanes
    indices = linear_assignment(dist_mat)
    #matches, unmatched_new, unmatched_old = [], [], [] 
    #for col in range(num_new_lanes): 
    #    if col not in indices[0]: 
    #        unmatched_new.append(col)
    #for row in range(num_old_lanes): 
    #    if row not in indices[1]: 
    #        unmatched_old.append(col)
    
    #update new lanes
    max_lane_id = max_lane_id+1
    update_new_lane3d = dict()
    update_old_lane3d = old_lanes3d
    for col in range(num_new_lanes): 
        key = new_laneids[col]
        value = new_lane3d[key]
        if col not in indices[0]: #unmatched 
            update_new_lane3d[str(max_lane_id)] = value
            update_old_lane3d[str(max_lane_id)] = value
            max_lane_id += 1
        else: 
            idx_col = np.where(indices[0]==col)
            idx_row = indices[1][idx_col[0]]
            key_in_old = old_laneids[idx_row[0]]
            value_in_old = old_lanes3d[key_in_old]
            update_new_lane3d[key_in_old] = value 
            update_old_lane3d[key_in_old] = value
    #add unmatched-old 
    #for row in range(num_old_lanes): 
    #    if row not in indices[1]: 
    #        key = old_laneids[row]
   #         value = old_lanes3d[key]
    #        update_old_lane3d[key] = value
    
    return update_new_lane3d, update_old_lane3d
def augument3d(pt3d, extrapolate_ratio=0.1, order=2): 
    num_pt = pt3d.shape[0]
    x_min, x_max = min(pt3d[:,0]),max(pt3d[:,0])
    y_min, y_max = min(pt3d[:,1]),  max(pt3d[:,1])
    x_range=  x_max - x_min
    y_range = y_max - y_min
    z_mean = pt3d[:,2].mean()
    if x_range>y_range: #along x axis
        coeff = np.polyfit(pt3d[:,0],pt3d[:,1], order)
        ixs1 = np.linspace(x_min-extrapolate_ratio*x_range, 
                          x_min,int(num_pt/2))
        poly = np.poly1d(coeff)
        iys1 = poly(ixs1) 
        ixs2 = np.linspace(x_max, 
                          x_max+extrapolate_ratio*x_range, 
                          int(num_pt/2))   
        iys2 = poly(ixs1)                    
    else: 
        coeff = np.polyfit(pt3d[:,1],pt3d[:,0], order)
        poly = np.poly1d(coeff)
        iys1 = np.linspace(y_min-extrapolate_ratio*y_range, 
                           y_min, int(num_pt/2))
        ixs1 = poly(iys1)
        iys2 = np.linspace(y_max, 
                           y_max+extrapolate_ratio*y_range, int(num_pt/2))
        ixs2 = poly(iys2)
    izs1 = np.ones((ixs1.shape[0], 1))*z_mean
    aug_pt1 = np.hstack((np.expand_dims(ixs1,1),
                        np.expand_dims(iys1,1),
                        izs1))
    izs2 = np.ones((ixs2.shape[0], 1))*z_mean
    aug_pt2 = np.hstack((np.expand_dims(ixs2,1),
                        np.expand_dims(iys2,1),
                        izs2))
    aug_pt = np.vstack((aug_pt1,pt3d,aug_pt2 ))
    return aug_pt


def augument2d(pt2d, ratio=0.1, order=2):
    #augment out of image region 
    num_pt = pt2d.shape[0]
    x_min, x_max = min(pt2d[:,0]),max(pt2d[:,0])
    y_min, y_max = min(pt2d[:,1]),  max(pt2d[:,1])
    x_range=  x_max - x_min
    coeff = np.polyfit(pt2d[:,0],pt2d[:,1], order)
    ixs1 = np.linspace(x_min-ratio*x_range, x_min,int(num_pt/2))
    ixs2 = np.linspace(x_max,x_max+ratio*x_range, int(num_pt/2))
    ixs = np.concatenate((ixs1, ixs2))
    poly = np.poly1d(coeff)
    iys = poly(ixs) 
    
    pt2d_aug = np.hstack((np.expand_dims(ixs,1),
                        np.expand_dims(iys,1)))    
    aug_pt = np.vstack((pt2d,pt2d_aug ))
    return aug_pt         

def sort_points(point_arr, yaxis=True): 
    #input: point_arr: Nx2 array
    if yaxis: 
        arr = point_arr[:,1]
    else: #x-axis
        arr = point_arr[:,0]
    indices = np.argsort(arr)
    out_arr = point_arr[indices[:],:]
    return out_arr

def resample_points(points, num, x_lim=None): 
    coeff = np.polyfit(points[:,0], points[:,1], 2)
    poly = np.poly1d(coeff)
    if x_lim is None: 
        xs  = np.linspace(min(points[:,0]), max(points[:,0]), num)
    else: 
        xs = np.linspace(x_lim[0], x_lim[1], num)
    ys = poly(xs)
    return np.hstack((np.expand_dims(xs,1),np.expand_dims(ys,1)))

def resample_points_yaxis(points, num, y_lim=None): 
    coeff = np.polyfit(points[:,1], points[:,0], 2)
    poly = np.poly1d(coeff)
    if y_lim is None: 
        ys = np.linspace(min(points[:,1]), max(points[:,1]), num)
    else: 
        ys = np.linspace(y_lim[0], y_lim[1], num)
    xs = poly(ys)
    return np.hstack((np.expand_dims(xs,1),np.expand_dims(ys,1)))

def extend_road(src_points, length, num_samples=10, y_axis=True):
    #append road in front and end with length, 
    #suppose the road has been sorted
    
    if y_axis: 
        coeff_front = np.polyfit(src_points[:10,1], src_points[:10,0], 2)
        poly_front = np.poly1d(coeff_front)
        coeff_end = np.polyfit(src_points[:-10,1], src_points[:-10,0],2)
        poly_end = np.poly1d(coeff_end)
        y_start, y_end =src_points[0,1], src_points[-1,1]
        if (y_start -src_points[1,1])<=0:
            y_fake_start = y_start-length
        else: 
            y_fake_start = y_start+length
        y_fake_front_samples = np.linspace(y_fake_start, y_start, num_samples)
        x_fake_front_samples = poly_front(y_fake_front_samples)
        
        if (y_end - src_points[-2,1]) >=0:
            y_fake_end = y_end + length
        else: 
            y_fake_end = y_end - length 
        y_fake_end_samples = np.linspace( y_end,y_fake_end, num_samples)
        x_fake_end_samples = poly_end(y_fake_end_samples)
        
    else: #x-axis
        coeff_front = np.polyfit(src_points[:10,0], src_points[:10,1], 2)
        poly_front = np.poly1d(coeff_front)
        coeff_end = np.polyfit(src_points[:-10,0], src_points[:-10,1],2)
        poly_end = np.poly1d(coeff_end)
        x_start, x_end =src_points[0,0], src_points[-1,0]
        if (x_start -src_points[1,0])<=0:
            x_fake_start = x_start-length
        else: 
            x_fake_start = x_start+length
        x_fake_front_samples = np.linspace(x_fake_start, x_start, num_samples)
        y_fake_front_samples = poly_front(x_fake_front_samples)
        if (x_end - src_points[-2,0]) >=0:
            x_fake_end = x_end + length
        else: 
            x_fake_end = x_end - length 
        x_fake_end_samples = np.linspace( x_end,x_fake_end, num_samples)
        y_fake_end_samples = poly_end(x_fake_end_samples)

    fake_front_samples = np.transpose(np.vstack((x_fake_front_samples, y_fake_front_samples)))
    fake_end_samples = np.transpose(np.vstack((x_fake_end_samples, y_fake_end_samples)))
    output_samples = np.concatenate((fake_front_samples, src_points, fake_end_samples))
    return output_samples

def lane_3d_reconstruct(raw_lane_det2d,  
                        camera_intrinsic, pose_list, depth_list, \
                        winsize =20, overlap_winsize=2, 
                        point_num=5):##frame_list,
         
    image_size = (depth_list[0].shape[1], depth_list[0].shape[0])#(w,h)
    num_frame = len(raw_lane_det2d)
    #step 1. 2d-3d raw points 
    pt3d_list = []
    pt2d_list = []
    for i in range(num_frame): 
        pose  = pose_list[i]
        depth = depth_list[i]
        raw_curr_lane2d = raw_lane_det2d[i]
        if len(raw_curr_lane2d)<1: #no lane 
            pt3d_list.append(np.array([]))
            pt2d_list.append(np.array([]))
        else: 
            temp2d = []
            temp3d = []
            for line in raw_curr_lane2d: 
                do_augument=False
                ds = [depth[int(line[j,1]), int(line[j,0])] for j in range(line.shape[0])]
                line3d = project_2d_to_3d_depth_arr(np.transpose(line),\
                                            camera_intrinsic, pose, np.array(ds))
                line3d = np.transpose(line3d)#(Nx3)
                if do_augument: 
                    aug_line3d = augument3d(line3d, extrapolate_ratio=0.1)
                    aug_line2d = project_3d_to_2d(np.transpose(aug_line3d), intrisic, pose)
                    aug_line2d = np.transpose(aug_line2d)
                    line2d = aug_line2d
                    line3d = aug_line3d
                else: 
                    line2d = line
                temp2d.append(line2d)
                temp3d.append(line3d)
            temp2d = np.vstack(temp2d)
            temp3d = np.vstack(temp3d)
            if 0: #debug: check detection and projection 
                import os
                proj_temp2d = project_3d_to_2d(np.transpose(temp3d), camera_intrinsic, pose)
                raw_pt2d = np.vstack(raw_curr_lane2d)
                canvas = cv2.imread(os.path.join('/home/lidanzha/hdd/work/CSG/videos/kitti_images', '04', 'image_2','%06d.png'%i))
                for p in range(raw_pt2d.shape[0]):
                    cv2.circle(canvas, (int(raw_pt2d[p,0]), int(raw_pt2d[p,1])), 7, (0,0,255), -1)
                for p in range(proj_temp2d.shape[1]):
                    cv2.circle(canvas, (int(proj_temp2d[0,p]), int(proj_temp2d[1,p])), 3, (0,255,0), -1) 
                cv2.imwrite('test/reproj_%06d.jpg'%i, canvas)
             
            pt3d_list.append(temp3d)
            pt2d_list.append(temp2d)

    #Step2. run sliding window and cluster on selected frames 
    frame_list = list(range(num_frame))
    windows_index = sliding_window(frame_list, window_size=winsize, overlap_size=overlap_winsize)
    old_lanes3d = None #previous lane, used for tracking
    out_lanes2d = dict() #output 
    max_lane_id = -1
    for j, win_idx in enumerate(windows_index): 
        select_pt3d = pt3d_list[win_idx[0]]
        select_frames = [frame_list[win_idx[0]]]
        select_poses = [pose_list[win_idx[0]]]
        select_depth = [depth_list[win_idx[0]]]
        for i in range(1, len(win_idx)): 
            if pt3d_list[win_idx[i]].size>0: 
                if select_pt3d.size<=0: 
                    select_pt3d = pt3d_list[win_idx[i]]
                else: 
                    select_pt3d = np.concatenate((select_pt3d, pt3d_list[win_idx[i]]), axis=0)
            select_frames.append(frame_list[win_idx[i]])
            select_poses.append(pose_list[win_idx[i]])
            select_depth.append(depth_list[win_idx[i]])
        lanes3d_frames = cluster3d(select_pt3d)
        if old_lanes3d is None:
            old_lanes3d = lanes3d_frames
        else: #associate line ids 
            lanes3d_frames, old_lanes3d = track_lanes(lanes3d_frames, old_lanes3d)
        lanes2d_frames = reproject3d_to_frames(lanes3d_frames, image_size, select_frames,
                                        camera_intrinsic, select_poses, select_depth,
                                        point_num)
        if 0: #debug, check cluster result
            plt.clf()
            for k,v in lanes3d_frames.items(): 
                plt.scatter(v[:,0], v[:,1], label=k)
            plt.legend()
            saver = 'test/cluster_%06d_%06d.jpg'%(select_frames[0], select_frames[-1])
            plt.savefig(saver)
            import os
            for k,v in lanes2d_frames.items(): 
                canvas = cv2.imread(os.path.join('/home/lidanzha/hdd/work/CSG/videos/kitti_images', '04', 'image_2','%06d.png'%int(k)))
                for k2,v2 in v.items():
                    for p in range(v2.shape[0]):
                        cv2.circle(canvas, (int(v2[p,0]), int(v2[p,1])), 7, (0,255,0), -1) 
                saver = 'test/final_%06d.jpg'%int(k)
                cv2.imwrite(saver, canvas)

        #todo: overlap frames, current, just copy &covering
        for k,v in lanes2d_frames.items(): 
            out_lanes2d[int(k)] = copy.deepcopy(v)
            lane_id_list = list(map(int, v.keys()))
            max_lane_id = max(max_lane_id, max(lane_id_list))
    
    
    #Step3. refine in 2d; with raw deteceion results(high precision)
    final_lane2d = dict()
    minimal_lane_distance = 35
    max_lane_id += 1
    for frame_id in range(num_frame):            
        det_lane2d = raw_lane_det2d[frame_id]
        det_indices = list(range(len(det_lane2d)))
        gen_keys = []
        gen_lane2d = []
        for k,v in out_lanes2d[frame_id].items(): 
            gen_keys.append(int(k))
            gen_lane2d.append(v)
        num_detection = len(det_indices)
        num_generation = len(gen_keys)
        if num_detection < 1: 
            lane2d = dict()
            #remove near lanes
            dist_mat = np.zeros((num_detection, num_detection))
            selected =[]
            selected.append(0)
            lane2d[gen_keys[0]] = sort_points(gen_lane2d[0])
            for i in range(1, len(gen_keys)): 
                min_dist = 1000
                for j in selected: 
                    dist = calculate_lane_distance(gen_lane2d[i], gen_lane2d[j])
                    min_dist = min(dist, min_dist)
                if min_dist >= minimal_lane_distance: 
                    lane2d[gen_keys[i]] = sort_points(gen_lane2d[i])
                    selected.append(i)
            final_lane2d[frame_id] = lane2d
            continue 
        if num_generation < 1: #det number always>=1
            lane2d = dict()
            for l in det_lane2d: #create new lane(incremental id)
                lane2d[max_lane_id] = sort_points(l)
                max_lane_id += 1
            final_lane2d[frame_id] = lane2d
            continue 

        dist_mat = np.zeros((num_generation, num_detection))
        for i in range(num_generation): 
            for j in range(num_detection): 
                dist_mat[i,j] = calculate_lane_distance(gen_lane2d[i], det_lane2d[j])
        indices = linear_assignment(dist_mat)
        matches, unmatched_gen, unmatched_det = [], [], [] 
        for col in range(num_detection): 
            if col not in indices[1]: 
                unmatched_det.append(col)
        for row in range(num_generation): 
            if row not in indices[0]: 
                unmatched_gen.append(row)
        for i, row in enumerate(indices[0]): 
            col = indices[1][i]
            matches.append((row, col))
        lane2d = dict()
        for (row, col) in matches: 
            temp_lane = resample_points(det_lane2d[col], point_num)
            lane2d[gen_keys[row]] = sort_points(temp_lane)#use raw detection, since more accurate
        for i in unmatched_det:
            temp_lane = resample_points(det_lane2d[i], point_num)
            lane2d[max_lane_id] = sort_points(temp_lane)
            max_lane_id += 1 
        for i in unmatched_gen:
            #check is noisy or true 
            min_dist = 1000
            for key, seleced_lane in lane2d.items(): 
                dist = calculate_lane_distance(gen_lane2d[i], seleced_lane)
                min_dist = min(min_dist, dist)
            if min_dist > minimal_lane_distance: 
                lane2d[gen_keys[i]] = sort_points(gen_lane2d[i])
            #else: 
            #    print(frame_id, '--noisy:', min_dist)
        if 0: 
            canvas = cv2.imread(os.path.join('/home/lidanzha/hdd/work/CSG/videos/kitti_images', '04', 'image_2','%06d.png'%int(frame_id)))
            for k,v in lane2d.items():
                for p in range(v.shape[0]):
                    cv2.circle(canvas, (int(v[p,0]), int(v[p,1])), 7, (0,255,0), -1)
                cv2.putText(canvas, str(k), (int(v[0, 0]), int(v[0,1])),
                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv2.LINE_AA)
            saver = 'test/final_%06d.jpg'%int(frame_id)
            cv2.imwrite(saver, canvas)
            
            
        final_lane2d[frame_id] = lane2d


    return final_lane2d


if __name__=='__main__': 
    import pickle
    input = pickle.load(open('../../debug_lane.pkl', 'rb'))
    det_lanes = input[0]
    camera_intrinsic = input[1]
    pose_list = input[2]
    pose_list = pose_list[:31]
    depth_list = input[3]
    lane2d, lane3d  = lane_3d_reconstruct(det_lanes,
										  camera_intrinsic,
										  pose_list,
										  depth_list)
