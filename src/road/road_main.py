import numpy as np
from src.ui.image_utils import convert_pixel_to_canvas, convert_canvas_to_pixel, normTransMat
import src.road.GenerateCurRoad as Road
from src.road.lane import sort_points,resample_points_yaxis, extend_road
from src.convert_util import project_2d_to_3d_depth_arr
import copy

def initialize_bounary_points(image_raw, scale, pad): 
	#currently use templates to initialzie 4 boundary points
	#TODO: auto detection road area and locate boundary poitns and detect lane number 

	pts = []
	w,h = image_raw.shape[1], image_raw.shape[0]
	width_gridsize = w/6
	height_gridsize = h/6
	pts.append([int(width_gridsize*3), int(height_gridsize*2)])#north
	pts.append([int(width_gridsize*2), int(height_gridsize*3)])#west
	pts.append([int(width_gridsize*3), int(height_gridsize*4)])#south
	pts.append([int(width_gridsize*4), int(height_gridsize*3)])#east
	canvas_pts = convert_pixel_to_canvas(pts, scale, pad)
	canvas_pts_dict ={'North': canvas_pts[0], 'West': canvas_pts[1], 'South':canvas_pts[2], 'East': canvas_pts[3]}
	return canvas_pts_dict

def road_bound(camera_param, bound_pts_pixels): 
	trsnsMat = camera_param.Mat_P[:, (0, 1, 3)]
	trsnsMat = np.asarray(trsnsMat, dtype=np.float64)
	Ex, Ey, Wx, Wy, Sx, Sy, Nx, Ny = 0, 0, 0, 0, 0, 0, 0, 0
	for order in ['North', 'South', 'East', 'West']:
		[x,y] = bound_pts_pixels[order]
		temp = normTransMat(np.dot(np.linalg.inv(trsnsMat), np.array([x, y, 1]).reshape((3, 1))))
		if order == 'East':
			Ex=-temp[0][0]/1000
		elif order == 'West':
			Wx=-temp[0][0]/1000
		elif order == 'North':
			Ny=-temp[1][0]/1000
			Nx=-temp[0][0]/1000
		elif order == 'South':
			Sy=-temp[1][0]/1000
			Sx=-temp[0][0]/1000
	Ey = (Ny+Sy)/2
	Wy = (Ny+Sy) / 2

	return [[Ex,Ey],[Wx,Wy],[Nx,Ny],[Sx,Sy]]


def generate_road(camera_param, bound_pts, lane_num, 
				  raw_image_shape, scale, pad,
				  xodr_filename,roadpts_N,roadpts_W,roadpts_S,roadpts_E): 
	#bound_pts, lan_num: dict, key=directions
	#need raw_image? -> need resize to predefined resolution?
	CALIBRATION_SHAPE=(1920,1080)
	width_scale = CALIBRATION_SHAPE[0]/raw_image_shape[0]
	height_scale = CALIBRATION_SHAPE[1]/raw_image_shape[1]
	bound_pts_pixels = dict()
	bound_pts_pixels['North'] = convert_canvas_to_pixel([bound_pts['North']], scale, pad)[0]
	bound_pts_pixels['West'] = convert_canvas_to_pixel([bound_pts['West']], scale, pad)[0]
	bound_pts_pixels['South'] = convert_canvas_to_pixel([bound_pts['South']], scale, pad)[0]
	bound_pts_pixels['East']= convert_canvas_to_pixel([bound_pts['East']], scale, pad)[0]

	bound_pts_pixels_calib = copy.deepcopy(bound_pts_pixels)
	for o in ['North', 'West', 'East', 'South']: 
		bound_pts_pixels_calib[o][0] *= width_scale
		bound_pts_pixels_calib[o][1] *= height_scale
		

	road_x_range= road_bound(camera_param, bound_pts_pixels_calib)#bound_pts_pixels)
	LaneCount = np.array([[lane_num['en'], lane_num['es']], #east left, east right
				  [lane_num['wn'], lane_num['ws']], #west left, west right
				  [lane_num['nw'], lane_num['ne']], #north left, north right
				  [lane_num['sw'], lane_num['se']] #south left, south right
				 ],	 dtype=np.uint8)
	boundPts = np.array(road_x_range)
	
	sample_set=[roadpts_N,roadpts_W,roadpts_S,roadpts_E]
	for i in range(4):
		if sample_set[i] :
			sample_flag= 1 
			sample_input= sample_set[i]
	Road.GenerateIntersection(xodr_filename, \
							  boundPts,\
							  np.array([40, 40, 40, 40]).reshape((4, 1)),\
							  LaneCount,sample_flag,sample_input)

	return bound_pts_pixels
    

def generate_egoview_straight_road(lane2d_dict, center_lane_id, 
							  intrinsic, poses, depths, 
							  xodr_filename): 
	#todo: need check all frames has lane labels?
	
	center_lane_world_coordinates = []
	left_lane_num = []
	right_lane_num = []
	num_frame = min(len(lane2d_dict.keys()), min(len(poses), len(depths)))
	for i in range(num_frame): 
		curr_lane2d = lane2d_dict[i]
		depth = depths[i]
		pose = poses[i]
		#project center line 
		#for ego-view, the direction is always y-ascending direction
		ctr_pt2d = sort_points(curr_lane2d[center_lane_id])
		ds = [depth[int(ctr_pt2d[j,1]), int(ctr_pt2d[j,0])] for j in range(ctr_pt2d.shape[0])]		
		ctr_pt3d = project_2d_to_3d_depth_arr(np.transpose(ctr_pt2d),intrinsic, pose, np.array(ds))
		ctr_pt3d = np.transpose(ctr_pt3d)
		left_num = 0
		right_num = 0 
		for lane_id, lane_pts in curr_lane2d.items(): 
			if lane_id != center_lane_id: 
				pt2d = curr_lane2d[lane_id]
				coeff = np.polyfit(pt2d[:,1], pt2d[:,0],2)
				poly = np.poly1d(coeff)
				x = poly(ctr_pt2d[:, 1])
				if x[0]<ctr_pt2d[0,0]: 
					left_num +=1
				else: 
					right_num += 1
		left_lane_num.append(left_num)
		right_lane_num.append(right_num)
		center_lane_world_coordinates.append(ctr_pt3d)

	#check if the road structure changed, 
	left_diff = np.diff(left_lane_num)
	right_diff = np.diff(right_lane_num)
	if sum(abs(left_diff)) + sum(abs(right_diff)) <1: 
		#one lane 
		driving_lane_count = [left_lane_num[0], right_lane_num[0]]
		sidewalk_lane_count = [0,0]
		roadID = '1'
		sample_pts = np.vstack(center_lane_world_coordinates)
		sample_pts = sample_pts[:,:2]
		sample_pts = sort_points(sample_pts)
		interval_y = np.diff(sample_pts[:,1])
		max_y_speed = np.max(abs(interval_y))
		new_sample_pts = resample_points_yaxis(sample_pts, sample_pts.shape[0])
		#append some road at front and end, assure all vehicles in the road at anytime
		ext_sample_pts = extend_road(new_sample_pts, num_frame*max_y_speed, num_samples=num_frame)
		Road.GenerateRoad(ext_sample_pts,xodr_filename,roadID,driving_lane_count,sidewalk_lane_count)
	else: 
		#divide into segments
		pass