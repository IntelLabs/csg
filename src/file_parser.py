#file parsing 
# 
import os
import csv
import numpy as np
import binascii
from scipy.spatial.transform import Rotation as R_util
from src.track.trajectory import split_fitting, linear_interp
from src.convert_util import project_2d_to_3d_depth
import xml.etree.ElementTree as ET
from src.road.GenerateCurRoad import prettyXml
from src.track.trajectory import smooth_trajectory_kalman
import math

    
def csv_read_matrix(file_path, delim=',', comment_str="#"):   
    
    with open(file_path,'r') as f:
        reader = csv.read(file_path, delimiter=delim)
        mat = [row for row in reader]

    return mat

def read_kitti_poses_file(file_path):

    raw_mat = csv_read_matrix(file_path)
    if len(raw_mat) > 0 and len(raw_mat[0]) != 12:
        raise print("KITTI pose files must have 12 entries per row "
                 "and no trailing delimiter at the end of the rows (space)")
    try:
        mat = np.array(raw_mat).astype(float)
    except ValueError:
        raise print('error when parsing kitti file')
    # yapf: disable
    poses = [np.array([[r[0], r[1], r[2], r[3]],
                       [r[4], r[5], r[6], r[7]],
                       [r[8], r[9], r[10], r[11]],
                       [0, 0, 0, 1]]) for r in mat]

    return poses#PosePath3D(poses_se3=poses) 
def read_kittti_calib_file(file_path): 
    with open(file_path, 'r') as f: 
        raw_lines = f.readlines()
    p0_line = raw_lines[0]
    p2_line = raw_lines[2]
    tr_line = raw_lines[4]
    
    data_line = p0_line[len('P0: '):-1]
    data_str_arr = data_line.split(' ')
    data_arr =np.asarray(data_str_arr, float) 
    p0_mat = np.reshape(np.asarray(data_arr).flatten(), (3,4))

    data_line = p2_line[len('P2: '):-1]
    data_str_arr = data_line.split(' ')
    data_arr =np.asarray(data_str_arr, float)
    p2_mat = np.reshape(np.asarray(data_arr).flatten(), (3,4))

    data_line = tr_line[len('Tr: '):-1]
    data_str_arr = data_line.split(' ')
    data_arr =np.asarray(data_str_arr, float)
    tr_mat = np.reshape(np.asarray(data_arr).flatten(), (3,4))

    return p0_mat[:3,:3]#p0_mat, p2_mat, tr_mat

def read_kitti_timestamp_file(file_path): 
    """
    parses timestamp file in KITTI format 
    :param file_path: the trajectory file path (or file handle)
    :return: list of timestamps
    """
    raw_timestamps_mat = csv_read_matrix(file_path)
    error_msg = ("timestamp file must have one column of timestamps and same number of rows as the KITTI poses file")
    try:
        timestamps_mat = np.array(raw_timestamps_mat).astype(float)
    except ValueError:
        raise print(error_msg)
    return timestamps_mat

def parse_calib_file(file_name):
    values = []
    with open(file_name, 'r') as f: 
        line = f.readline()
        strss= line.split(' ')
        for s in strss:
            s = s.strip()
            if len(s)<1: 
                continue 
            values.append(float(s))
        arr = np.array(values).reshape((3,3))
    return arr
    
def parse_slam_file(file_name, time_file): 
    #return dict of poses, key =frame_id
    #time_file = os.path.join(kitti_seq_folder, 'times.txt')
    with open(time_file, 'r') as f:
        ts_list = f.readlines()
    for i in range(len(ts_list)): 
        t = float(ts_list[i].strip())
        ts_list[i] = float("{:10.6f}".format(t))
    with open(file_name,'r') as f:
        raw_lines=f.readlines()

    poses = []
    frame_ids = []
    for line in raw_lines:
        temp = line.strip().split(' ')
        fid = ts_list.index(float(temp[0]))
        tx,ty,tz = temp[1], temp[2], temp[3]
        q0,q1,q2,w = temp[4], temp[5], temp[6],temp[7]
        r = R_util.from_quat([q0, q1, q2, w])
        try: 
            r_mat = r.as_matrix() #support: scipy>=1.4.0
        except: #scipy<1.4.0
            r_mat = r.as_dcm()

        temp_mat = np.eye(4)
        temp_mat[:3,:3] = r_mat
        temp_mat[0,3] = tx
        temp_mat[1,3] = ty
        temp_mat[2,3] = tz
        frame_ids.append(fid)
        poses.append( temp_mat)

    return frame_ids,poses

def parse_slam_file_any(file_name): 
    #return dict of poses, key =frame_id
    with open(file_name,'r') as f:
        raw_lines=f.readlines()

    poses = []
    frame_ids = []
    for line in raw_lines:
        temp = line.strip().split(' ')
        fid = float(temp[0])
        tx,ty,tz = temp[1], temp[2], temp[3]
        q0,q1,q2,w = temp[4], temp[5], temp[6],temp[7]
        r = R_util.from_quat([q0, q1, q2, w])
        try:
            r_mat = r.as_matrix()
        except:
            r_mat = r.as_dcm()

        temp_mat = np.eye(4)
        temp_mat[:3,:3] = r_mat
        temp_mat[0,3] = tx
        temp_mat[1,3] = ty
        temp_mat[2,3] = tz
        frame_ids.append(fid)
        poses.append( temp_mat)

    return frame_ids,poses
def interp_slam_keyframe(slam_poses, slam_valid_frames): 

    full_poses = []
    full_frames = []
    full_poses.append(slam_poses[0])
    full_frames.append(slam_valid_frames[0])
    
    for i in range(1,len(slam_valid_frames)): 
        ts, te = slam_valid_frames[i-1], slam_valid_frames[i]
        if (te - ts) ==1: 
            pass
        else: 
            interval = te - ts
            try: 
                r_s = R_util.from_matrix(slam_poses[i-1][:3,:3])
                r_e = R_util.from_matrix(slam_poses[i][:3,:3])
            except: 
                r_s = R_util.from_dcm(slam_poses[i-1][:3,:3])
                r_e = R_util.from_dcm(slam_poses[i][:3,:3])
            euler_s = r_s.as_euler('xyz')
            euler_e = r_e.as_euler('xyz')
            tran_s = slam_poses[i-1][:3,3]
            tran_e = slam_poses[i][:3,3]
            for j in range(interval-1): 
                interp_fid = ts + j+1
                interp_euler = euler_s + (euler_e-euler_s)/interval * (j+1)
                interp_tran = tran_s + (tran_e - tran_s)/interval*(j+1)
                interp_r = R_util.from_euler('xyz',interp_euler)
                try:
                    interp_rmat = interp_r.as_matrix()
                except:
                    interp_rmat = interp_r.as_dcm()
                temp = np.eye(4)
                temp[:3,:3] = interp_rmat
                temp[:3,3] = interp_tran
                full_frames.append(interp_fid)
                full_poses.append(temp)
        full_poses.append(slam_poses[i])
        full_frames.append(te)
    
    if 0: #plot to see
        rx,ry,rz = [],[],[]
        tx,ty,tz = [],[],[]
        for item in full_poses: 
            try:
                rmat = R_util.from_matrix(item[:3,:3])
            except: 
                rmat = R_util.from_dcm(item[:3,:3])
            r_angle = rmat.as_euler('xyz')
            rx.append(r_angle[0])
            ry.append(r_angle[1])
            rz.append(r_angle[2])
            tx.append(item[0,-1])
            ty.append(item[1,-1])
            tz.append(item[2,-1])
        f, (ax1,ax2) = plt.subplots(1,2)
        ax1.plot(range(len(full_poses)), rx)
        ax1.plot(range(len(full_poses)), ry)
        ax1.plot(range(len(full_poses)), rz)
        
        ax2.plot(range(len(full_poses)), tx)
        ax2.plot(range(len(full_poses)), ty)
        ax2.plot(range(len(full_poses)), tz)
        plt.show()
    return full_poses, full_frames

def calculate_speed(xyz1, xyz2): 
    return np.sqrt((xyz1[0]-xyz2[0])**2 + (xyz1[1]-xyz2[1])**2)
def speed_converter(meter_sec): 
    return meter_sec*3.6#return km/h

def extend_traj_unseen_frame(raw_traj, raw_frame_list, ttl_frame, do_end_interp=True): 
    lost_frame = list(set(range(ttl_frame)) - set(raw_frame_list))
    if len(lost_frame)<1: #full Traj
        return raw_traj
    full_traj = []
    full_frames = []

    #propagate to lost frames, with same speed
    #todo: here we suppose raw_traj is continuous
    start_frame_id = raw_frame_list[0]
    end_frame_id = raw_frame_list[-1]
    if start_frame_id>0: #always interp on the starting frames
        step = raw_traj[1] - raw_traj[0]
        for j in range(start_frame_id): 
            full_traj.append(raw_traj[0] - step*(start_frame_id-j)) 
            full_frames.append(j)
    #add orginal traj
    for j in range(len(raw_frame_list)): 
        full_traj.append(raw_traj[j])
        full_frames.append(raw_frame_list[j])

    if end_frame_id < ttl_frame-1 and do_end_interp: 
        step = raw_traj[-1] - raw_traj[-2]
        for j in range(0, ttl_frame - end_frame_id): 
            full_traj.append(raw_traj[-1]+(j+1)*step)
            full_frames.append(j+end_frame_id+1)
    return np.vstack(full_traj)
def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))     

'''
argument: auto_fill: if set True, will extrapolate trajectory across all frames
'''
def export_traj_xml_file_ego(other_traj, intrinsic, pose, depth,
                             xml_filename, scenario_name, fps, auto_fill=1):
    num_frames = min(len(pose), len(depth))
    model_list = ["vehicle.lincoln.mkz2017", "vehicle.yamaha.yzf", "vehicle.tesla.model3", 
                  "vehicle.audi.tt", "vehicle.dodge_charger.police", "vehicle.chevrolet.impala",
                  "vehicle.mini.cooperst","vehicle.citroen.c3","vehicle.seat.leon"]
    
    #format: x,y, z, speed
    ego_traj = []
    xyz_index = [0,2,1]

    for i in range(num_frames): 
        xzy = pose[i][:3,3]
        xyz = xzy[xyz_index]
        if i != num_frames-1: 
            xzy2 = pose[i+1][:3,3]
            speed = calculate_speed(xyz, xzy2[xyz_index])#todo
            speed = speed_converter(speed*fps) #m/sec-> km/h
        ego_traj.append(np.append(xyz, speed))
    ego_traj = np.vstack(ego_traj)

    other_full_traj = []
    yaw_angles = []#calculate relative angle 

    for traj in other_traj: #traj: id, frame_id, x,y
        traj_3d = []
        frame_list = []
        for row in traj: 
            oid, fid,x,y = int(row[0]), int(row[1]), row[2], row[3]
            pt3d = project_2d_to_3d_depth(np.asarray([x,y]), intrinsic, pose[fid], 
                                        depth[fid][int(y), int(x)])
            traj_3d.append(pt3d)
            frame_list.append(fid)
        traj_3d = np.vstack(traj_3d)
        smooth_traj_3d = smooth_trajectory_kalman(traj_3d)
        smooth_traj_3d = split_fitting(smooth_traj_3d[:,:2])#traj_3d[:,:2])#smooth by poly fit
        #calculate yaw on raw trajectories
        dir_other = traj[2,:2]-traj[0,:2]
        dir_ego = ego_traj[2,:2] - traj[0,:2]
        radian = angle_between(dir_ego, dir_other)
        degree = math.degrees(radian)
        yaw_angles.append(int(degree/15)*15) #into bins

        if auto_fill==0: #no auto trajectory fill
            full_traj = smooth_traj_3d
        elif auto_fill==1: #only fill on front, no interpolation when vehicle is disappear 
            full_traj = extend_traj_unseen_frame(smooth_traj_3d, frame_list, num_frames, do_end_interp=False)
        else: #fill on both start and ending 
            full_traj = extend_traj_unseen_frame(smooth_traj_3d, frame_list, num_frames, do_end_interp=True)
        #append speed to full_traj 
        full_traj = np.append(full_traj, np.zeros([full_traj.shape[0],1]),1)
        for i in range(full_traj.shape[0]-1): 
            speed = np.sqrt(np.linalg.norm(full_traj[i+1]-full_traj[i]))#calculate_speed(full_traj[i+1], full_traj[i])
            full_traj[i,-1] = speed_converter(speed*fps)
        full_traj[-1,-1] =full_traj[-2, -1]  # speed_{final} =speed_{final-1}
        other_full_traj.append(full_traj)
    if 0: 
        import matplotlib.pyplot as plt
        for traj in other_full_traj: 
            plt.plot(traj[:,0], traj[:,1], 'b')
        plt.plot(ego_traj[:,0], ego_traj[:,1], 'r')
        plt.show()


    ego_yaw = 90
    precision_format = '%.6f'
    rootNode = ET.Element('scenarios')
    pure_scenario_name = os.path.basename(scenario_name)
    scenarioNode = ET.SubElement(rootNode, 'scenario', name=pure_scenario_name, town = "case2_map", type="CustomizedBasic")
    egoData = ET.SubElement(scenarioNode, 'ego_vehicle', model="vehicle.lincoln.mkz2017", velocity="%d"%(ego_traj[0][-1]), 
                            x=precision_format%(ego_traj[0][0]), y=precision_format%(ego_traj[0][1]), yaw='%.1f'%ego_yaw, z='1.0')
    egoTarget = ET.SubElement(scenarioNode, 'target', 
                            x=precision_format%(ego_traj[-1][0]), y=precision_format%(ego_traj[-1][1]), z="0.0")
    egoRoute = ET.SubElement(scenarioNode, 'route')
    for i in range(1, ego_traj.shape[0]-1): 
        xyzs = ego_traj[i,:]
        ET.SubElement(egoRoute, 'waypoint',  connection="RoadOption.LANEFOLLOW", 
                      speed = precision_format%(xyzs[-1]),
                      x = precision_format%(xyzs[0]), y = precision_format%(xyzs[1]),z='0.0')
    #for other actors
    for i, traj in enumerate(other_full_traj): 
        other_actor = ET.SubElement(scenarioNode, 'other_actor', model=model_list[(i+2)%9], 
                                    velocity="%d"%(traj[0,-1]), 
                                    x=precision_format%(traj[0, 0]), y=precision_format%(traj[0, 1]),
                                    yaw="%.1f"%(yaw_angles[i]+ego_yaw), z="1.0")
        other_target = ET.SubElement(other_actor,'other_target',
                                    x=precision_format%(traj[-1, 0]), y=precision_format%(traj[-1, 1]), 
                                    z='0.0') 
        other_route = ET.SubElement(other_actor, 'other_route')
        for j in range(1, traj.shape[0]-1): 
            xys = traj[j,:]
            ET.SubElement(other_route, 'waypoint',  connection="RoadOption.LANEFOLLOW", 
                        speed = precision_format%(xys[-1]),
                        x = precision_format%(xys[0]), y = precision_format%(xys[1]),z='0.0')

    prettyXml(rootNode, '\t', '\n')            #format normalization   
    #ET.dump(rootNode)  
    Newtree=ET.ElementTree(rootNode)
    #ET.dump(rootNode)  
    Newtree.write(xml_filename)


    
   