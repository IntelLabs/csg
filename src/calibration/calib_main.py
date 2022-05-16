import numpy as np
from src.calibration.VPLine import CVanLnSel
from src.calibration.CameraCal import CamCal
from src.ui.image_utils import get_pixel_coordinate
import copy


def camera_clibration_canvas(para_lines, ref_points_distance, raw_image_shape,
                             image_scale, image_paddings):
    # convert to image coordinates
    CALIBRATION_SHAPE=(1920,1080)
    FPts = []
    SegNodes = []
    NodeDist = []
    camcal_obj=CamCal()
    vpline = CVanLnSel()
    width_scale = CALIBRATION_SHAPE[0]/raw_image_shape[0]
    height_scale = CALIBRATION_SHAPE[1]/raw_image_shape[1]
    para_lines_copy = copy.deepcopy(para_lines)
    ref_points_distance_copy = copy.deepcopy(ref_points_distance)
    for paraline in para_lines_copy:
        for i in range(2):#2 lines 
            canvas_x, canvas_y = paraline[i][1], paraline[i][2]
            pixel_x, pixel_y = get_pixel_coordinate(canvas_x, canvas_y, image_scale, image_paddings)
            FPts.append(np.array([pixel_x*width_scale, pixel_y*height_scale]))#fit to calibration image 
            canvas_x, canvas_y = paraline[i][3], paraline[i][4]
            pixel_x, pixel_y = get_pixel_coordinate(canvas_x, canvas_y, image_scale, image_paddings)
            FPts.append(np.array([pixel_x*width_scale, pixel_y*height_scale]))#fit to calibration image 
    for ref_line in ref_points_distance_copy:
        for i in range(2):
            canvas_x, canvas_y = ref_line[1+2*i], ref_line[1+2*i+1]
            pixel_x, pixel_y = get_pixel_coordinate(canvas_x, canvas_y, image_scale, image_paddings)
            SegNodes.append(np.array([pixel_x*width_scale, pixel_y*height_scale]))
        NodeDist.append([float(ref_line[-1])])
    
    VPs = vpline.process(FPts)
    camcal_obj.process(VPs, SegNodes, NodeDist)

    return camcal_obj.CamParam

