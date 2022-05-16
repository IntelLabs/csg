
import numpy as np 
import math
import copy


def inverse_rigid_trans(Tr):
    """ Inverse a rigid body transform matrix (3x4 as [R|t])
        [R'|-R't; 0|1]
    """
    inv_Tr = np.zeros_like(Tr)  # 3x4
    inv_Tr[0:3, 0:3] = np.transpose(Tr[0:3, 0:3])
    inv_Tr[0:3, 3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
    return inv_Tr


def project_2d_to_3d_depth(image_pt,intrisicMat, extrisicMat, depthValue, reorder=True): 
    #ref: https://github.com/raulmur/ORB_SLAM2/issues/226
    u,v = image_pt[0].item(), image_pt[1].item()
    d = depthValue

    #fx = intrisicMat[0][0]
    #fy = intrisicMat[1][1]
    #cx = intrisicMat[0][2]
    #cy = intrisicMat[1][2]
    #x = ((u - cx)*d)/fx
    #y = ((v - cy)*d)/fy
    #pts_3d_ref = np.asarray([x,y,d]).reshape(3,1)
    twc = inverse_rigid_trans(extrisicMat) #T_wc
    rwc = twc[:3,:3]#extrisicMat[:3,:3]#inv_T[:3,:3]
    twc_vec = twc[:3,3].reshape(3,1)#extrisicMat[:3,3].reshape(3,1)
    p_c = np.dot(np.linalg.inv(intrisicMat), (np.asarray([u*d,v*d,d]).reshape(3,1)))
    p_world = np.dot(np.transpose(rwc), (p_c-twc_vec))

    #debug 
    if 0:
        homo_pt3d = np.array([p_world[0].item(), p_world[1].item(), p_world[2].item(), 1]) 
        uvw = np.dot(intrisicMat, np.dot(extrisicMat[:3,:], homo_pt3d))
        norm_uvw =[uvw[0]/uvw[2], uvw[1]/uvw[2], 1]
        #check diff between norm_uvw and image_pt

    #reshape to xyz-order
    if reorder: 
        xyz_index = [0,2,1]
        p_world = p_world[xyz_index].squeeze()
    else: 
        p_world = p_world.squeeze()
   
    return p_world

def project_2d_to_3d_depth_arr(pt2d_arr,
                          intrisicMat, extrisicMat, 
                          depth_arr, reorder=True): 
    #pt2d_arr: (2xN), each column indicates a point
    #return value: pt3d: (2xN)
    N = pt2d_arr.shape[1]
    homo_pt2d_arr = np.zeros((3,N))
    for i in range(N): 
        homo_pt2d_arr[0,i] = pt2d_arr[0,i]* depth_arr[i]
        homo_pt2d_arr[1,i] = pt2d_arr[1,i]* depth_arr[i]
        homo_pt2d_arr[2,i] =  depth_arr[i]
    twc = inverse_rigid_trans(extrisicMat) #T_wc
    rotMat = twc[:3,:3]
    transVec = twc[:3,3].reshape(3,1)
    transMat = np.repeat(transVec, N, axis=1)

    p_cam = np.dot(np.linalg.inv(intrisicMat), homo_pt2d_arr)
    p_cam_trans = p_cam - transMat
    p_world = np.dot(np.linalg.inv(rotMat), (p_cam - transMat))

    #debug 
    if 0:
        homo_pt3d = np.array([p_world[0].item(), p_world[1].item(), p_world[2].item(), 1]) 
        uvw = np.dot(intrisicMat, np.dot(extrisicMat[:3,:], homo_pt3d))
        norm_uvw =[uvw[0]/uvw[2], uvw[1]/uvw[2], 1]
        #check diff between norm_uvw and image_pt

    #reshape to xyz-order
    if reorder: 
        xyz_index = [0,2,1]
        p_world = p_world[xyz_index, :].squeeze()
    else: 
        p_world = p_world.squeeze()
   
    return p_world

def project_3d_to_2d(pt3d, intrisicMat, extrisicMat, reorder=True): 
    #intrinsicMat: 3x3
    #extrisicMat: 3x4[R|T]
    #pt3d: 3xn (each column represent a coordinate)

    #reorder
    if reorder: 
        xyz_index = [0,2,1]
        pt3d = pt3d[xyz_index,:]
    n = pt3d.shape[1]
    twc = inverse_rigid_trans(extrisicMat) #T_wc
    homo_pt3d = np.append(pt3d, np.ones((1, n)), axis=0)
    #uvw = np.dot(intrisicMat, np.dot(extrisicMat[:3,:], homo_pt3d))
    uvw = np.dot(intrisicMat, np.dot(twc[:3,:], homo_pt3d))
    uvw[0,:] /=uvw[2,:]
    uvw[1,:] /=uvw[2,:]
    uvw[2,:] = 1
    return uvw[:2,:]        


def cart2hom( pts_3d):
    """ Input: nx3 points in Cartesian
            Oupput: nx4 points in Homogeneous by pending 1
    """
    n = pts_3d.shape[0]
    pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
    return pts_3d_hom
def calculate_speed(p3d1, p3d2, frame_interval=1, fps=10): 
    # Input 3D coordinates 
    # return speed in km/h 

    dist = math.sqrt((p3d1[0]-p3d2[0])*(p3d1[0]-p3d2[0])+
                (p3d1[2]-p3d2[2])*(p3d1[2]-p3d2[2]))
    v_meter_sec = dist * fps/frame_interval
    v_km_hour = 3.6 * v_meter_sec
    return v_km_hour

def get_abs_pose(rela_pose, ref_pose): 
    #rela_pose: convert to current pose to referent_pose 
    #all pose is 4x4[R|T]

    prod = np.dot(rela_pose, ref_pose)
    return prod[:3,:]

def get_abs_slam_pose(slam_mat, sc): 
    abs_slam_mat = copy.deepcopy(slam_mat)
    sx,sy,sz = sc[0], sc[1],sc[2]
    for i in range(len(slam_mat)): 
        abs_slam_mat[i][0,-1] *= sx
        abs_slam_mat[i][1,-1] *= sy
        abs_slam_mat[i][2,-1] *= sz

    return abs_slam_mat

