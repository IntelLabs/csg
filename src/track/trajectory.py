import numpy as np
import scipy.interpolate as interp
import cv2
from tkinter import messagebox
from src.ui.image_utils import convert_pixel_to_canvas,convert_canvas_to_pixel,fit_image_to_canvas
#from ..ui.dialog import View_AskAutoInsertion, stop_signal #wrong: this import will make a copy
import src.ui.dialog as dlg
from .kcf.kcftracker import KCFTracker
from config import SOURCE
from pykalman import KalmanFilter

MIN_TRAJECTORY_LENGTH  = 5#10

import queue
import threading

#traj: array of [x,y], (num_pt,2)
#arg: s: controll smoothness(ie. fitting error), larger s-> smoother(e.g. 100, 200 or notset)
def split_fitting(pts, s=100): 
    num_points = pts.shape[0]
    degree = 2
    if num_points<=degree: 
        return pts
    # -spline  fitting---------
    duplicates = []
    for i in range(1, pts.shape[0]):
        if np.allclose(pts[i,:], pts[i-1,:]):
            duplicates.append(i)
    polyline = pts
    if duplicates:
        polyline = np.delete(polyline, duplicates, axis=0)
    if polyline.shape[0] <=degree: 
        return pts
    #print('number points: ', polyline.shape[0])
    tck, u = interp.splprep(polyline.T, k=degree)#, s=s)#100) 

    new_pts = interp.splev(u, tck)
    #recover from deleted duplicate points
    smooth_pts = np.zeros(pts.shape)
    idx = 0
    for i in range(num_points): 
        if i in duplicates: 
            smooth_pts[i,:] = smooth_pts[i-1,:]
        else: 
            smooth_pts[i,0] = new_pts[0][idx]
            smooth_pts[i,1] = new_pts[1][idx]
            idx += 1

    if 0:
        from matplotlib import pyplot as plt
        plt.scatter(pts[:,0], pts[:,1], edgecolors='blue')
        plt.scatter(smooth_pts[:, 0], smooth_pts[:, 1], edgecolor='red')
        u = np.linspace(0.0, 1.0, num_points)
        B = np.column_stack(interp.splev(u, tck))
        plt.scatter(B[:,0], B[:,1], edgecolor='yellow') #fitted curve)
        plt.show()
    
    return smooth_pts

def smooth_trajectory(traj_raw): 
    traj_smooth = []
    for item in traj_raw: 
        #print("item:", item)
        centers = item[:,2:]
        smooth_centers = split_fitting(centers) 
        traj_smooth.append(np.hstack((item[:,0:2], smooth_centers)))
    return traj_smooth

def smooth_trajectory_kalman(points_raw, auto_ks=True): 
    num_pt, dim = points_raw.shape[0], points_raw.shape[1]
    smooth_data = np.zeros(points_raw.shape)
    for col_idx in range(points_raw.shape[1]):
        observation = points_raw[:, col_idx]
        kf = KalmanFilter(transition_matrices= np.asarray([[1,1], [0,1]]), #[1,dt][0,dt]
                        initial_state_mean = [observation[0],0],
                        transition_covariance= 0.01*np.eye(2))
        if auto_ks: #fit kf parameters
            kf = kf.em(observation, n_iter=2) 
        (smoothed_state_means, smoothed_state_covariances) = kf.smooth(observation)
        smooth_data[:, col_idx] = smoothed_state_means[:,0].squeeze() #position, velocity
    if 0: 
        import matplotlib.pyplot as plt
        plt.figure(1)
        times = range(points_raw.shape[0])
        plt.plot(times, points_raw[:, 0], 'bo',
                times, points_raw[:, 1], 'ro',
                times, smooth_data[:, 0], 'm.',#'b--',
                times, smooth_data[:, 1], 'k.')#'r--',)
        plt.show()
    return smooth_data


#box_inputs: list of [(object_id, object_label_id, x1,y1,x2,y2, score, source)]
#output: list of object [(object_id, frame_id, center_x,center_y]
def generate_trajectory(box_inputs): 
    #if no objects is labeled in any frame?
    frame_list = range(0, len(box_inputs))
    target_ids =[]
    target_pos = []
    for i,frame_data in enumerate(box_inputs): 
        for j, object_data in enumerate(frame_data): 
            target_ids.append(object_data[0])
            target_pos.append([i,j])

    unq_target_ids = np.unique(target_ids)
    trajectory = []
    for id in unq_target_ids: 
        pos_indices = np.where(np.asarray(target_ids)==id)[0]
        curr_traj = np.zeros((len(pos_indices), 4))#num_framex4 
        for k, pos in enumerate(pos_indices): 
            i,j = target_pos[pos]
            raw_item = box_inputs[i][j]
            frame_id = frame_list[i]
            cx = round((raw_item[2]+raw_item[4])/2)
            cy = round((raw_item[3]+raw_item[5])/2)
            curr_traj[k,:]=id, frame_id, cx,cy
        trajectory.append(curr_traj)
    return smooth_trajectory(trajectory)
def remove_short_trajectory(objectList, objectList_pixel): 
    raw_traj = generate_trajectory(objectList)
    del_object_id = []
    for traj in raw_traj: 
        if traj.shape[0]<MIN_TRAJECTORY_LENGTH: 
            del_object_id.append(traj[0,0])
    for frame_id, frame_data_pixel in enumerate(objectList_pixel): 
        for  j, object_data_pixel in enumerate(frame_data_pixel):
            curr_oid = int(object_data_pixel[0])
            if curr_oid in del_object_id: 
                del objectList_pixel[frame_id][j]
                del objectList[frame_id][j]
    return objectList, objectList_pixel


#return: 
#target_ids: list of unique target object ids
#target_pos: list of array for each id : frame_id, object_idx_in_frame, object_label x1,y1,x2,y2
def collect_trajectory_info(objects_pixel): 
    target_ids =[]
    target_pos = []
    for i,frame_data in enumerate(objects_pixel): 
        for j, object_data in enumerate(frame_data): 
            target_ids.append(object_data[0])
            target_pos.append([i,j,object_data[1], object_data[2], object_data[3], object_data[4], object_data[5]]) 
    #group by id 
    traj_by_ids = []
    unq_ids = np.unique(target_ids)
    for id in unq_ids: 
        indices = np.where(np.asarray(target_ids)==id)[0]
        selected = [target_pos[i] for i in indices]
        traj_by_ids.append(np.vstack(selected))

    return list(unq_ids), traj_by_ids 
#add $interp_pt_num points between $start and $end

def linear_interp(start,end, interp_pt_num): 
    interval = (end-start)/(interp_pt_num+1)
    out = np.zeros((2+interp_pt_num, 1))
    out[0] = start
    for i in range(interp_pt_num): 
        out[i+1] = start + interval*(i+1)
    out[-1] = end
    return out
#interp arr_in along axis 
def linear_interp_2d(arr_in, interp_pt_num, axis=0): 
    if axis==0: #interp on rows, column number is not changed
        arr_out = np.zeros((arr_in.shape[0]+interp_pt_num, arr_in.shape[1]), dtype=arr_in.dtype)
        for cid in range(arr_in.shape[1]): 
            col_in = arr_in[:,cid]
            col_out = linear_interp(col_in[0], col_in[-1], interp_pt_num)
            arr_out[:,cid] = col_out.ravel()
    else: 
        arr_out = np.zeros((arr_in.shape[0], arr_in.shape[1]+interp_pt_num), dtype=arr_in.type)
        for rid in range(arr_in.shape[0]): 
            row_in = arr_in[rid, :]
            row_out = linear_interp(row_in[0], row_in[-1], interp_pt_num)
            arr_out[rid,:] = row_out.ravel()
    return arr_out
def postprocessing_trajectory(vcap, tracker, #here tracker is deepsort, changed to kcf?
                               objectList,objectList_pixel, 
                               imgScale, imgPadding,
                               do_backward=False,do_reindex=True): 
    unq_traj_ids, traj_info_id = collect_trajectory_info(objectList_pixel)
    #1. interpolate to complete traj 
    
    source = SOURCE['Auto']
    for i in range(len(unq_traj_ids)): 
        frames = traj_info_id[i][:,0]
        diffs =[k-j for j, k in zip(frames[:-1], frames[1:])] 
        nonones = [idx for idx, v in enumerate(diffs) if v !=1 ]
        if len(nonones) >0: 
            traj_info_id_bak = traj_info_id[i]
            for j in nonones: 
                interp_frame_num = int(diffs[j])-1
                out = linear_interp_2d(traj_info_id[i][j:j+2, :], interp_frame_num)
                for elem in out[1:-1,:]: 
                    fid = int(elem[0])
                    elem[1] = len(objectList_pixel[fid])
                    objectList_pixel[fid].append([unq_traj_ids[i], int(elem[2]), elem[-4], elem[-3], elem[-2],elem[-1], 0, source])
                    canvas_xys = convert_pixel_to_canvas([[elem[-4], elem[-3], elem[-2],elem[-1]]], imgScale, imgPadding)[0]
                    objectList[fid].append([unq_traj_ids[i], int(elem[2]), canvas_xys[0], canvas_xys[1], canvas_xys[2],canvas_xys[3], 0, source])
                    traj_info_id_bak = np.concatenate((traj_info_id_bak, np.reshape(elem,(1,7))))
                   # print('insert item:', objectList[fid][-1]) 
            #reorder by frame-ids?
            traj_info_id_bak = traj_info_id_bak[np.argsort(traj_info_id_bak[:,0])]
            traj_info_id[i] = traj_info_id_bak
    
    #2. remove too short traj
    remove_indices = []
    remove_traj_indices = []
    for i, traj in enumerate(traj_info_id): 
        if len(traj)< MIN_TRAJECTORY_LENGTH: 
            for elem in traj: 
                remove_indices.append(np.array((elem[0], elem[1])))
            remove_traj_indices.append(i)
    
    for i in reversed(remove_traj_indices):
        del unq_traj_ids[i]
        del traj_info_id[i]
    if len(remove_indices) >0: 
        remove_indices = np.asarray(remove_indices)
        remove_frames = np.unique(remove_indices[:,0])
        for f in remove_frames: 
            idx = np.where(remove_indices[:,0] == f)[0]#locate frame id
            oids = remove_indices[idx[:], 1].flatten()#locate object id
            reverse_oids = np.sort(oids)[::-1]#delete in revserse
            for i in reverse_oids: 
                del objectList_pixel[int(f)][int(i)]
                del objectList[int(f)][int(i)]
    
    #3. backward-tracking
    if do_backward: #use backward tracking or not
        img_width  = vcap.get(3) 
        img_height = vcap.get(4) 
        roi_area = [int(img_width*1/6), int(img_height*1/6), 
                    round(img_width*5/6+0.5), round(img_height*5/6+0.5)]
        warmup_frame_num = 3
        tracker.reset()
        for i, traj_data in enumerate(traj_info_id): 
            first_fid = int(traj_data[0,0])
            if first_fid>= warmup_frame_num: 
                cx = (traj_data[0,-4]+traj_data[0,-2])/2
                cy = (traj_data[0,-3]+traj_data[0,-1])/2
                curr_oid = unq_traj_ids[i]
                if (cx>roi_area[0]) and (cy>roi_area[1]) and(cx<roi_area[2]) and (cy<roi_area[3]): 
                    print('backward for object id: ', curr_oid)
                    tracker.reset() #reset to clean reid features
                    #init
                    for idx in range(min(10, traj_data.shape[0]-1), -1, -1): #for speed, only use 5 frames to kf
                        elem = traj_data[idx,:]
                        frame_id = int(elem[0])
                        vcap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
                        _, image = vcap.read()
                        input = [elem[2], elem[-4], elem[-3], elem[-2], elem[-1], 0.5]#fake confidence =0,5
                        tracker.update([input], image, frame_id, force_matching=True)
                    #back-track
                    for k in range(first_fid-1, -1, -1):
                        #print('processing frame:', k)
                        vcap.set(cv2.CAP_PROP_POS_FRAMES, k) 
                        _, image = vcap.read() 
                        tracker.update([[]], image, k, force_matching=True) 
                        curr_trk_rst = tracker.get_current_result(consider_time=False)
                        if len(curr_trk_rst)<=0: 
                            break
                        else: 
                            score = 0#
                            tr = curr_trk_rst[0]
                            ltrb = convert_pixel_to_canvas([[tr[3], tr[4], tr[5], tr[6]]], imgScale, imgPadding)[0]
                            objectList[k].append([curr_oid, int(tr[1]), ltrb[0], ltrb[1], ltrb[2], ltrb[3], score, source])
                            objectList_pixel[k].append([curr_oid, int(tr[1]), tr[3], tr[4], tr[5], tr[6], score, source])
    
    
    #4.re-map the object_id(tobe contingous)
    max_object_id = 0
    if do_reindex: 
        id_map = {v:index+1 for index,v in enumerate(unq_traj_ids)}
        for fi in range(len(objectList)): 
            for oi in range(len(objectList[fi])):
                objectList[fi][oi][0] = id_map[objectList[fi][oi][0]]
                objectList_pixel[fi][oi][0] = id_map[objectList_pixel[fi][oi][0]]
                max_object_id = max(objectList[fi][oi][0], max_object_id)

    #5. generate&smooth traj. for final output 
    trajectory = generate_trajectory(objectList)
    return trajectory, objectList, objectList_pixel, max_object_id

#return array(num_pts, 2)
def select_trajectory(traj_in, object_id, frame_id): 
    #get traj of object_id till frame_id
    for item in traj_in: 
        if int(item[0,0]) == object_id:
            indices = np.where(item[:,1]<=frame_id)[0]
            centers = item[indices[:], 2:]
            return centers
        else: 
            continue
    return None

def kcf_track(videoCap, queue, curr_fid, bbox_pixel, stop_event):
    margin_factor = 0.3
    ttl_frames =int( videoCap.get(7))
    #todo: use track factory
    kcf_tracker = KCFTracker(True, False, True)
    videoCap.set(cv2.CAP_PROP_POS_FRAMES, curr_fid) 
    _, image = videoCap.read()
    width, height = image.shape[1], image.shape[0]
    x,y = bbox_pixel[0], bbox_pixel[1]
    w, h = bbox_pixel[2]-bbox_pixel[0]+1, bbox_pixel[3]-bbox_pixel[1]+1

    kcf_tracker.init([x,y,w,h], image, 0) #fake label, not used 
    for fid in range(curr_fid+1, ttl_frames): #track in all-rest
        if  stop_event.is_set():
            #print('stopped')
            with queue.mutex:
                queue.queue.clear()
            return
        videoCap.set(cv2.CAP_PROP_POS_FRAMES, fid)
        _, image = videoCap.read() 
        roi = kcf_tracker.update(image) #[x,y,w,h]
        cv2.rectangle(image, (int(roi[0]), int(roi[1])), (int(roi[0]+roi[2]-1), int(roi[1]+roi[3]-1)), 
                     (0,0,255), 2)
        x1 = max(0, int(roi[0]-margin_factor*roi[2]))
        y1 = max(0, int(roi[1]-margin_factor*roi[3]))
        x2 = min(width-1, int(roi[0]+roi[2]+margin_factor*roi[2]))
        y2 = min(height-1, int(roi[1]+roi[3]+margin_factor*roi[3]))
        cropped_bgr = image[y1:y2, x1:x2,:]
        data_dict = {'frame_id':fid, 'image':cropped_bgr, 'bbox':[roi[0], roi[1], roi[0]+roi[2]-1, roi[1]+roi[3]-1]}

        queue.put(data_dict)

def updateGUI(master, q, ws): 
    if not q.empty(): 
        data_dict = q.get()
        ws.insert(data_dict)
    master.after(10, lambda w=master,que=q, widget=ws: updateGUI(w,que, widget))

#only 1 item with 1 object id is added 
def add_with_check(raw_obj_list, raw_obj_pixel_list, 
                   add_fids, add_items, add_items_pixel): 
    final_obj_list = raw_obj_list 
    final_obj_pixel_list = raw_obj_pixel_list
    conflict_ids = []
    add_id = add_items[0][0]
    max_obj_id = add_id
    for i, fid in enumerate(add_fids): #do not check all frames 
        curr_oids = [item[0] for item in raw_obj_list[fid]]
        if add_id in curr_oids: 
            idx = curr_oids.index(add_id)
            if not raw_obj_list[fid][idx][-1]==SOURCE['Manual']: 
                conflict_ids.append([fid, idx])
    for frame_data in raw_obj_list: 
        curr_oids = [item[0] for item in frame_data]
        if len(curr_oids)>0: 
            max_obj_id = max(max(curr_oids), max_obj_id)
    if len(conflict_ids)>=1: 
        new_id = max_obj_id+1#new id assigned 
        #reassign new id 
        for conflict_item in conflict_ids: 
            fid, idx = conflict_item
            final_obj_list[fid][idx][0] = new_id
            final_obj_pixel_list[fid][idx][0] = new_id
        max_obj_id = new_id
    #todo? need to consider overlap to remove highly-overlapped targets?
    for i, fid in enumerate(add_fids): 
        final_obj_list[fid].append(add_items[i])
        final_obj_pixel_list[fid].append(add_items_pixel[i])
    return final_obj_list, final_obj_pixel_list, max_obj_id


'''
Input: object_item format same with objectList format(canvas coord): 
    [object_id, label_id, x1,y1,x2,y2, 1.0, 2] #1.0=confidence, 2=manual
    op can be ['insert', 'delete']
'''
def auto_refine(videoCap, root, curr_fid, object_item,
                objectList, objectList_pixel, trajectory, 
                imageScale, imagePadding, op, 
                old_object_item=None):
    ttl_frames = int(videoCap.get(7))
    max_obj_id=-1
    if op =='insert': #insert a new target from user
        #track the inesrted item from curr_fid+1 to ttl_frames-
        #create 2 thread: 1. for pop-up window, 2. for tracking
        
        stop_event= threading.Event()
        image_num = int(ttl_frames - curr_fid-1)
        ws = dlg.View_AskAutoInsertion(root, image_num, stop_event)
        bbox_pixel = convert_canvas_to_pixel([[object_item[2], object_item[3], object_item[4], object_item[5]]], 
                                            imageScale, imagePadding)[0]
        image_queue = queue.Queue()
        t = threading.Thread(target=kcf_track, args=(videoCap, image_queue, curr_fid, bbox_pixel, stop_event))
        t.start()

        updateGUI(root, image_queue, ws)
        root.wait_window(ws.top)
        if ws.done:
            is_selected = ws.thumbView.is_selected
            raw_data_dict = ws.thumbView.data
            #add current insert first
            added_items = []
            added_items_pixel =[]
            added_fids = []
            object_item_pixel = [object_item[0], object_item[1], 
                                 bbox_pixel[0], bbox_pixel[1], bbox_pixel[2], bbox_pixel[3], 
                                 1.0,  SOURCE['Manual']]
            added_fids.append(curr_fid)
            added_items.append(object_item)
            added_items_pixel.append(object_item_pixel)
            source = SOURCE['Track']
            for i, hit in enumerate(is_selected): #only 1 object is added in each frame
                if not hit: 
                    continue
                fid = curr_fid+i+1
                bbox_pixel = raw_data_dict[i]['bbox']#x1,y1,x2,y2
                object_id, label_id = object_item[0], object_item[1]
                bbox_canvas = convert_pixel_to_canvas([[bbox_pixel[0],bbox_pixel[1],bbox_pixel[2],bbox_pixel[3]]],
                                                    imageScale, imagePadding)[0]
                added_fids.append(fid)
                added_items_pixel.append([object_id, label_id, 
                                    bbox_pixel[0],bbox_pixel[1],bbox_pixel[2],bbox_pixel[3],
                                    1.0, source])
                added_items.append([object_id, label_id, 
                                    bbox_canvas[0],bbox_canvas[1],bbox_canvas[2],bbox_canvas[3],
                                    1.0, source])
            objectList, objectList_pixel, max_obj_id = add_with_check(objectList, objectList_pixel, 
                                                          added_fids, added_items, added_items_pixel)
        else: #cancel is selected 
            objectList[curr_fid].append(object_item)
            object_item_pixel = [object_item[0], object_item[1], 
                                 bbox_pixel[0], bbox_pixel[1], bbox_pixel[2], bbox_pixel[3], 
                                 1.0, SOURCE['Manual']]
            objectList_pixel[curr_fid].append(object_item_pixel)

        trajectory = generate_trajectory(objectList) #just smooth, no other post-processings

    elif op =='delete': 
        old_object_id = old_object_item[0] #object_item is not used 
        msgbox = messagebox.askquestion("Ask", "Do you want to delete in all frames?",icon='warning')
        if msgbox =='yes': 
            end_fid = ttl_frames
        else: 
            end_fid = curr_fid+1
        #start_fid = curr_fid #start from current to all
        start_fid = 0 #search all frames
        for fid in range(start_fid, end_fid): 
            frame_data = objectList[fid]
            for iid in range(len(frame_data)-1, -1, -1): 
                if frame_data[iid][0] == old_object_id: 
                    del objectList[fid][iid]
                    del objectList_pixel[fid][iid]
                    break
        trajectory = generate_trajectory(objectList)#zld: no need to reorder the object id
    elif op =='modify':#here modify indicates label or target_id
        old_object_id, old_label_id = old_object_item[0], old_object_item[1]
        new_object_id, new_label_id = object_item[0], object_item[1]
        if old_object_id!=new_object_id or old_label_id != new_label_id: 
            msgbox = messagebox.askquestion("Ask", "Do you want to modify in all frames?",icon='warning')
            if msgbox =='yes': 
                end_fid = ttl_frames
            else: 
                end_fid = curr_fid+1 
            #start_fid = curr_fid #start from current to all
            start_fid = 0 #search all frames
            for fid in range(start_fid, end_fid): 
                frame_data = objectList[fid]
                for iid in range(0, len(frame_data)): 
                    if frame_data[iid][0] == old_object_id: 
                        objectList[fid][iid][0] = new_object_id
                        objectList[fid][iid][1] = new_label_id
                        objectList_pixel[fid][iid][0] = new_object_id
                        objectList_pixel[fid][iid][1] = new_label_id
        trajectory = generate_trajectory(objectList)
    else: 
        print('Not supported op: ', op)

 
    return objectList, objectList_pixel, trajectory, max_obj_id