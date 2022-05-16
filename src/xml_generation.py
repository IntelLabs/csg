import pickle
import xml.etree.ElementTree as ET
import numpy as np
import math


def normTransMat(x):
    num = x.shape[1]
    y = np.zeros((3, num))
    for i in range(0, num):
        y[0, i] = x[0, i] / x[2, i]
        y[1, i] = x[1, i] / x[2, i]
        y[2, i] = 1
    return y

def prettyXml(element, indent, newline, level = 0):  
    if element:   
        if element.text == None or element.text.isspace():   
            element.text = newline + indent * (level + 1)    
        else:  
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)  
 
    temp = list(element) 
    for subelement in temp:  
        if temp.index(subelement) < (len(temp) - 1):  
            subelement.tail = newline + indent * (level + 1)  
        else:   
            subelement.tail = newline + indent * level  
        prettyXml(subelement, indent, newline, level = level + 1) 

def traj_prepare(videoFilename):
    anno_path = videoFilename+'_annotation.pkl'
    anno_raw = pickle.load(open(anno_path, 'rb'))
    cam_path = videoFilename+'_camcalib.pkl'
    raw_cam = pickle.load(open(cam_path, 'rb'))

    total_trajectory= [] 
    traj_pixel = anno_raw[2]
    trsnsMat = raw_cam.Mat_P[:, (0, 1, 3)]
    trsnsMat = np.asarray(trsnsMat, dtype=np.float64)

    for traj in traj_pixel: 
        pixel_arr = traj[:, 2:]
        traj_w=[]
        for wp in pixel_arr:
            x=wp[0]
            y=wp[1]
            wp_temp = normTransMat(np.dot(np.linalg.inv(trsnsMat), np.array([x, y, 1]).reshape((3, 1))))
            wp_temp=wp_temp.reshape((1, 3))
            traj_w.append([wp_temp[0][0],wp_temp[0][1]])
        total_trajectory.append(traj_w)
       
    return total_trajectory
    
        
def convert2XML(videoFilename):
    
    total_trajectory = traj_prepare(videoFilename)
    actor_num=len(total_trajectory)-1
    rootNode=ET.Element('scenarios')
    subScenNode=ET.SubElement(rootNode,'scenario', name=videoFilename+"_00", type="CustomizedBasic", town=videoFilename+"_map")
    ego_traj=total_trajectory[0]
    ego=ET.SubElement(subScenNode,'ego_vehicle', x=str(ego_traj[0][0]), y=str(ego_traj[0][1]), z="0.0", yaw="270", model="vehicle.lincoln.mkz2017", velocity="60")  
    ego_tar= ET.SubElement(subScenNode,'target', x=str(ego_traj[-1][0]), y=str(ego_traj[-1][1]), z="0.0")
    ego_route= ET.SubElement(subScenNode,'route')
    for i in range(len(ego_traj)-1):
        ego_wayp= ET.SubElement(ego_route,'waypoint',x=str(ego_traj[i+1][0]), y=str(ego_traj[i+1][1]) ,z="0.0", connection="RoadOption.LANEFOLLOW", speed="68")

    for k in range(actor_num):
        actor_traj=total_trajectory[1+k]
        actor= ET.SubElement(subScenNode,'other_actor', x=str(actor_traj[0][0]), y=str(actor_traj[0][1]), z="0.0")
        actor_tar= ET.SubElement(actor,'other_target', x=str(actor_traj[-1][0]), y=str(actor_traj[-1][1]), z="0.0")
        actor_route= ET.SubElement(actor,'other_route')
        for t in range(len(actor_traj)-1):
            actor_wayp= ET.SubElement(actor_route,'waypoint',x=str(actor_traj[t+1][0]), y=str(actor_traj[t+1][1]),z="0.0", connection="RoadOption.LANEFOLLOW", speed="68")

    prettyXml(rootNode, '\t', '\n')
    Newtree=ET.ElementTree(rootNode) 
    Newtree.write(videoFilename+'_TrafficDynamics.xml')
