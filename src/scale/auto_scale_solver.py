
from __future__ import absolute_import, division, print_function
import math

import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F

class BackprojectDepth(nn.Module):
    """Layer to transform a depth image into a point cloud
    """
    def __init__(self, batch_size, height, width):
        super(BackprojectDepth, self).__init__()

        self.batch_size = batch_size
        self.height = height
        self.width = width

        meshgrid = np.meshgrid(range(self.width), range(self.height), indexing='xy')
        self.id_coords = np.stack(meshgrid, axis=0).astype(np.float32)
        self.id_coords = nn.Parameter(
            torch.from_numpy(self.id_coords),
            requires_grad=False)

        self.ones = nn.Parameter(
            torch.ones(self.batch_size, 1, self.height * self.width),
                       requires_grad=False)

        self.pix_coords = torch.unsqueeze(torch.stack(
            [self.id_coords[0].view(-1), self.id_coords[1].view(-1)], 0), 0)
        self.pix_coords = self.pix_coords.repeat(batch_size, 1, 1)
        self.pix_coords = nn.Parameter(
            torch.cat([self.pix_coords, self.ones], 1), requires_grad=False)

    def forward(self, depth, inv_K):
        cam_points = torch.matmul(inv_K[:, :3, :3], self.pix_coords)
        cam_points = depth.view(self.batch_size, 1, -1) * cam_points
        cam_points = torch.cat([cam_points, self.ones], 1).reshape(
            self.batch_size, 4, self.height, self.width)

        return cam_points



class ScaleRecovery(nn.Module):
    """Layer to estimate scale through dense geometrical constrain
    """
    def __init__(self, batch_size, height, width):
        super(ScaleRecovery, self).__init__()
        self.backproject_depth = BackprojectDepth(batch_size, height, width)
        self.batch_size = batch_size
        self.height = height
        self.width = width
        self.ground_masks = []

    # derived from https://github.com/zhenheny/LEGO
    def get_surface_normal(self, cam_points, nei=1):
        cam_points_ctr  = cam_points[:, :-1, nei:-nei, nei:-nei]
        cam_points_x0   = cam_points[:, :-1, nei:-nei, 0:-(2*nei)]
        cam_points_y0   = cam_points[:, :-1, 0:-(2*nei), nei:-nei]
        cam_points_x1   = cam_points[:, :-1, nei:-nei, 2*nei:]
        cam_points_y1   = cam_points[:, :-1, 2*nei:, nei:-nei]
        cam_points_x0y0 = cam_points[:, :-1, 0:-(2*nei), 0:-(2*nei)]
        cam_points_x0y1 = cam_points[:, :-1, 2*nei:, 0:-(2*nei)]
        cam_points_x1y0 = cam_points[:, :-1, 0:-(2*nei), 2*nei:]
        cam_points_x1y1 = cam_points[:, :-1, 2*nei:, 2*nei:]

        vector_x0   = cam_points_x0   - cam_points_ctr
        vector_y0   = cam_points_y0   - cam_points_ctr
        vector_x1   = cam_points_x1   - cam_points_ctr
        vector_y1   = cam_points_y1   - cam_points_ctr
        vector_x0y0 = cam_points_x0y0 - cam_points_ctr
        vector_x0y1 = cam_points_x0y1 - cam_points_ctr
        vector_x1y0 = cam_points_x1y0 - cam_points_ctr
        vector_x1y1 = cam_points_x1y1 - cam_points_ctr

        normal_0 = F.normalize(torch.cross(vector_x0,   vector_y0,   dim=1), dim=1).unsqueeze(0)
        normal_1 = F.normalize(torch.cross(vector_x1,   vector_y1,   dim=1), dim=1).unsqueeze(0)
        normal_2 = F.normalize(torch.cross(vector_x0y0, vector_x0y1, dim=1), dim=1).unsqueeze(0)
        normal_3 = F.normalize(torch.cross(vector_x1y0, vector_x1y1, dim=1), dim=1).unsqueeze(0)

        normals = torch.cat((normal_0, normal_1, normal_2, normal_3), dim=0).mean(0)
        normals = F.normalize(normals, dim=1)

        refl = nn.ReflectionPad2d(nei)
        normals = refl(normals)

        return normals

    def get_ground_mask(self, cam_points, normal_map, threshold=5):
        b, _, h, w = normal_map.size()
        cos = nn.CosineSimilarity(dim=1, eps=1e-6)

        threshold = math.cos(math.radians(threshold))
        ones, zeros = torch.ones(b, 1, h, w).cuda(), torch.zeros(b, 1, h, w).cuda()
        vertical = torch.cat((zeros, ones, zeros), dim=1)

        cosine_sim = cos(normal_map, vertical).unsqueeze(1)
        vertical_mask = (cosine_sim > threshold) | (cosine_sim < -threshold)

        y = cam_points[:,1,:,:].unsqueeze(1)
        ground_mask = vertical_mask.masked_fill(y <= 0, False)

        return ground_mask

    def forward(self, depth, K, real_cam_height):
        inv_K = torch.inverse(K)

        cam_points = self.backproject_depth(depth, inv_K)
        surface_normal = self.get_surface_normal(cam_points)
        ground_mask = self.get_ground_mask(cam_points, surface_normal)

        cam_heights = (cam_points[:,:-1,:,:] * surface_normal).sum(1).abs().unsqueeze(1)
        cam_heights_masked = torch.masked_select(cam_heights, ground_mask)
        cam_height = torch.median(cam_heights_masked).unsqueeze(0)

        scale = torch.reciprocal(cam_height).mul_(real_cam_height)

        self.ground_masks.append(ground_mask)

        return scale
    def get_depth_scale(self, depth_list, K, real_cam_height): 
        pred_scales = []
        tensor_camera_height = torch.tensor([real_cam_height]).cuda()
        tensor_K = K.copy()
        tensor_K = torch.from_numpy(tensor_K).unsqueeze(0).float().cuda()
        abs_depth = []
        for pred_depth in depth_list: 
            tensor_depth = torch.from_numpy(pred_depth).unsqueeze(0).cuda()
            s = self.forward(tensor_depth,tensor_K, tensor_camera_height)
            scale_np = s.cpu().item()
            pred_scales.append(scale_np)
            abs_depth.append(pred_depth*scale_np)
        depth_ratio = np.median(np.asarray(pred_scales))
        return  depth_ratio, abs_depth

    def get_object_mask(self, obj_list): 
        mask = np.ones((self.height, self.width), dtype=bool)
        margin = 10
        for obj in obj_list: 
            x1,y1,x2,y2 = obj[2], obj[3], obj[4], obj[5]
            x1 = int(max(0, x1-margin))
            y1 = int(max(0, y1-margin ))
            x2 = int(min(self.width-1,  x2+margin))
            y2 = int(min(self.height-1, y2+ margin))
            mask[y1:y2, x1:x2] = False
        return mask
    

    '''
    not used    
    def get_translation_scale(self, videoCapture, K, depth_list, raw_pose, objectList): 
        #here depth_list is absolute depth, scale derived from above
        import cv2
        orb = cv2.ORB_create()
        frame_num = len(depth_list)
        top_n = 100#5
        #solve: A * x = B
        A_all = []
        B_all = []
        A_all_z = []#only optimize in z-direction
        B_all_z = []
        diffs = []
        for fid in range(0, frame_num-1, 2):
            videoCapture.set(cv2.CAP_PROP_POS_FRAMES, fid)
            _, img1 = videoCapture.read()
            videoCapture.set(cv2.CAP_PROP_POS_FRAMES, fid+1)
            _, img2 = videoCapture.read()
            kp1, des1 = orb.detectAndCompute(img1,None)
            kp2, des2 = orb.detectAndCompute(img2,None)
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            # Match descriptors.
            matches = bf.match(des1,des2)
            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)
            ground_mask1 = self.ground_masks[fid].cpu().numpy().squeeze()
            ground_mask2 = self.ground_masks[fid+1].cpu().numpy().squeeze()
            non_object_mask1 = self.get_object_mask(objectList[fid])
            non_object_mask2 = self.get_object_mask(objectList[fid+1])
            if 0:  
                img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:top_n],None,
                            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                cv2.imshow("", img3)
                cv2.waitKey(-1)
                cv2.imwrite("test.jpg", img3)
            #solve for scaler

            R1 = raw_pose[fid][:3,:3]
            R2 = raw_pose[fid+1][:3,:3]
            T1 = raw_pose[fid][:3,-1]
            T2 = raw_pose[fid+1][:3,-1]
            R1_inv = np.transpose(R1)#np.linalg.inv(R1)
            R2_inv = np.transpose(R2)#np.linalg.inv(R2)
            K_inv = np.linalg.inv(K)
            A = np.zeros((3,3))
            for r in range(3): 
                for c in range(3): 
                    A[r,c] = R1_inv[r,c]*T1[c] - R2_inv[r,c]*T2[c]
            A_z = A[[0,2],:]
            #A_z = np.zeros((2,3))
            #for c in range(3): 
            #    A_z[0,c] = R1_inv[r,c]*T1[c] -R2_inv[r,c]*T2[c]
            sample_num = 0
            for i in range(min(top_n, len(matches))): 
                img1_idx = matches[i].queryIdx
                img2_idx = matches[i].trainIdx
                (u1,v1) = kp1[img1_idx].pt
                (u2,v2) = kp2[img2_idx].pt
                #u1,v1 = int(u1), int(v1)
                #u2,v2 = int(u2), int(v2)
                #if (ground_mask1[int(v1),int(u1)] == False) or (ground_mask2[int(v2),int(u2)] ==False): 
                #    continue
                if (non_object_mask1[int(v1),int(u1)] == False) or (non_object_mask2[int(v2),int(u2)] ==False): 
                    continue
                if matches[i].distance >15: 
                    continue 
                if 0: 
                    import copy
                    canvas1 = copy.deepcopy(img1)
                    canvas2 = copy.deepcopy(img2)
                    cv2.circle(canvas1, (int(u1),int(v1)), 5, (0,255,0),thickness=-1)
                    cv2.circle(canvas2, (int(u2),int(v2)), 5, (0,255,0),thickness=-1)
                    cv2.imwrite("debug/img_%d_%d_1.jpg"%(fid, sample_num), canvas1)
                    cv2.imwrite("debug/img_%d_%d_2.jpg"%(fid, sample_num), canvas2)
                
                d1 = depth_list[fid][int(v1), int(u1)]
                d2 = depth_list[fid+1][int(v2), int(u2)]
                if abs(d1-d2)>1: 
                    continue 
                print('distance: ', matches[i].distance)
                print('depth: 1:', d1, '2:', d2)
                from src.convert_util import project_2d_to_3d_depth, project_2d_to_3d_depth_arr
                gt = np.array([8.09901665, 29.54727539, 27.75993509])
                pose1 = copy.deepcopy(raw_pose[fid])
                pose2 = copy.deepcopy(raw_pose[fid+1])
                pose1[0,-1] = pose1[0,-1]*gt[0]; pose1[1,-1] = pose1[1,-1]*gt[1]; pose1[2,-1]= pose1[2,-1]*gt[2]
                pose2[0,-1] = pose2[0,-1]*gt[0]; pose2[1,-1] = pose2[1,-1]*gt[1]; pose2[2,-1]= pose2[2,-1]*gt[2]
                
                world1 = project_2d_to_3d_depth(np.array([u1,v1]), K, pose1, d1, reorder=False)
                world2 = project_2d_to_3d_depth(np.array([u2,v2]), K, pose2, d2, reorder=False)
                diffs.append(world1-world2)

                B1 = np.dot(R1_inv, np.dot(K_inv, (np.asarray([u1*d1,v1*d1,d1]).reshape(3,1))))
                B2 = np.dot(R2_inv, np.dot(K_inv, (np.asarray([u2*d2,v2*d2,d2]).reshape(3,1))))
                B = B1-B2

                B_z_1 = np.dot(R1_inv[[0,2],:], np.dot(K_inv, (np.asarray([u1*d1,v1*d1,d1]).reshape(3,1))))
                B_z_2 = np.dot(R2_inv[[0,2],:], np.dot(K_inv, (np.asarray([u2*d2,v2*d2,d2]).reshape(3,1))))
                B_z = B_z_1 - B_z_2

                B_all.append(B)
                B_all_z.append(B_z)

            
                retval, local_x = cv2.solve(A, B, flags=cv2.DECOMP_SVD)
                print('local solution:', local_x.flatten())
                print('gt world 1:', world1.flatten(), '2:', world2.flatten())
                local_loss1 = np.square(np.linalg.norm(np.dot(A, gt) - B))
                local_loss2 = np.square(np.linalg.norm(np.dot(A, local_x) - B))
                print('loss, gt:', local_loss1, 'pred:', local_loss2)

                sample_num += 1
            if sample_num>0: 
                for _ in range(sample_num): 
                    A_all.append(A)
                    A_all_z.append(A_z)
        
        A_all = np.vstack(A_all)
        B_all = np.vstack(B_all)
        A_all_z = np.vstack(A_all_z)
        B_all_z = np.vstack(B_all_z)
        gt = np.array([8.09901665, 29.54727539, 27.75993509])
        #from scipy.optimize import lsq_linear
        #temp = lsq_linear(A_all, B_all)
        retval, X1 = cv2.solve(A_all, B_all, flags=cv2.DECOMP_SVD)
        retval, X2 = cv2.solve(A_all, B_all, flags=cv2.DECOMP_QR)
        loss1 = np.square(np.linalg.norm(A_all.dot(gt) - B_all))
        loss2 = np.square(np.linalg.norm(A_all.dot(X1) - B_all))

        lb = np.ones((3))*(-30)
        ub = np.ones((3))*(30)
        from scipy.optimize import lsq_linear
        solution = lsq_linear(A_all, B_all.squeeze(), bounds=(0,30), lsmr_tol='auto', verbose=2)
        loss3 = np.square(np.linalg.norm(A_all.dot(solution.x) - B_all))
        
        solution_z = lsq_linear(A_all_z, B_all_z.squeeze(), bounds=(0,30), lsmr_tol='auto', verbose=2)
        return solution.x#np.array(X1).reshape(1,3)

    '''
            



