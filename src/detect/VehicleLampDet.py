import numpy as np
import cv2
from sklearn.cluster import MeanShift,KMeans,estimate_bandwidth



def Detect(roi):
    center=[0,0]
    #hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsv=roi
    red_lower1 = np.array([150, 43, 46])
    red_upper1 = np.array([185, 255, 255])
    red_lower2 = np.array([0, 43, 46])
    red_upper2 = np.array([10, 255, 255])

    mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
    mask = mask1 + mask2
    Xlist = []
    for i in range(mask.shape[0]):
        for j in range(mask.shape[1]):
            if mask[i, j] == 255:
                Xlist.append((i, j))
    if Xlist:
        Xlist = np.asarray(Xlist)
        bandwidth = estimate_bandwidth(Xlist, quantile=0.33, n_samples=500)
        if bandwidth<=0:
            bandwidth=5
        clustering = MeanShift(bandwidth=bandwidth).fit(Xlist)
        Num_cluster = len(clustering.cluster_centers_)
        if Num_cluster>=2:
            kkk = np.zeros((Num_cluster, 1))
            for i in range(Num_cluster):
                temp = [x for x in clustering.labels_ if x == i]
                kkk[i, 0] = len(temp)/len(clustering.labels_)
            #kk = np.sum(kkk)
            max0=clustering.cluster_centers_[0]
            max1=clustering.cluster_centers_[1]
            center=np.array([(max0[1]+max1[1])/2, (max0[0]+max1[0])/2]) # [y,x]
            print(center)
        else:
            center=[0, 0]
    return center

