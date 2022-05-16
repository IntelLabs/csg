#error checking 
import numpy as np
low = np.s_[...,:2]
high = np.s_[...,2:]

def iou(A,B):
    A,B = A.copy(),B.copy()
    A[high] += 1; B[high] += 1
    intrs = (np.maximum(0,np.minimum(A[high],B[high])
                        -np.maximum(A[low],B[low]))).prod(-1)
    return intrs / ((A[high]-A[low]).prod(-1)+(B[high]-B[low]).prod(-1)-intrs)

def single_frame_check(objectlist, ov_thresh=0.7): 
    if len(objectlist)<=1: #only 1 object is given
        return 1 
    #1. duplicate id 
    ids = [item[0] for item in objectlist]
    unq_ids = list(set(ids))
    if len(ids)!=len(unq_ids): 
        return 0 
    #2. too overlapped
    boxes = [(item[2], item[3], item[4], item[5]) for item in objectlist]
    boxes = np.asarray(boxes)

    ovs = iou(boxes[:,None],boxes[None])
    N = boxes.shape[0]

    max_ov = max(np.asarray([ovs[i,j] for i in range(N) for j in range(i+1, N)]))
    if max_ov > ov_thresh: 
        return 0

    #others:? type consitencty across the frames?
    #center points->trjactory too close
    return 1

#return:
#1: pass validatoin test;0: fail in validation test
#-1: not valid data provided(empty arr)
def multi_frame_error_check(objectList): 
    valid_arr = []
    for frame_data in objectList: 
        if len(frame_data)>0: 
            valid = single_frame_check(frame_data)
        else: 
            valid = -1
        valid_arr.append(valid)
    return valid_arr