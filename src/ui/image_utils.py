import cv2
import numpy as np
from PIL import Image,ImageTk
import src.road.retinex as retinex
import copy
import random
import secrets

CALIBRATION_WIDTH = 1920
def normTransMat(x):
    num = x.shape[1]
    y = np.zeros((3, num))
    for i in range(0, num):
        y[0, i] = x[0, i] / x[2, i]
        y[1, i] = x[1, i] / x[2, i]
        y[2, i] = 1
    return y

#compatible to all resolutions, both horizontal and vertical
def fit_image_to_canvas(image, desired_width=640, desired_height=480, random_pad=False, padding_value=[0,0,0]):
    """Resizes an image keeping the aspect ratio mostly unchanged.

    Returns:
    image: the resized image
    window: (x1, y1, x2, y2). If max_dim is provided, padding might
        be inserted in the returned image. If so, this window is the
        coordinates of the image part of the full image (excluding
        the padding). The x2, y2 pixels are not included.
    scale: The scale factor used to resize the image
    padding: Padding added to the image [left, top, right, bottom]
    """
    # Default window (x1, y1, x2, y2) and default scale == 1.
    h, w = image.shape[0], image.shape[1]
    #import pudb;pudb.set_trace()   
    desired_ap = desired_height/desired_width
    image_ap = h/w 
    if image_ap<=desired_ap: 
        new_width = desired_width
        new_height = int((desired_width*h)/w )
    else: 
        new_width = int((desired_height*w)/h)
        new_height = desired_height

    width_scale = new_width / w
    height_scale = new_height / h
    scale = min(width_scale, height_scale)

    # Resize image using bilinear interpolation
    if scale != 1:
        image = cv2.resize(image, (round(w * scale), round(h * scale)))
    h,w, _ = image.shape
    y_pad = desired_height - h
    x_pad = desired_width - w
    secretGenerator = secrets.SystemRandom()
    top_pad = secretGenerator.randint(0, y_pad) if random_pad else y_pad // 2
    left_pad = secretGenerator.randint(0, x_pad) if random_pad else x_pad // 2

    padding = (left_pad, top_pad, x_pad - left_pad, y_pad - top_pad)
   # assert all([x >= 0 for x in padding])
    image = cv2.copyMakeBorder(image, top_pad, y_pad-top_pad, left_pad, x_pad-left_pad, cv2.BORDER_CONSTANT, value=padding_value)

    #image = functional.pad(image, padding)
    window = [left_pad, top_pad, w + left_pad, h + top_pad]

    return image, window, scale, padding

def resize_padding_image(image, scale, padding, padding_value =[0,0,0]):
    w, h = image.shape[1], image.shape[0] 
    if scale != 1: 
        image = cv2.resize(image, (round(w * scale),round(h * scale)))
    image = cv2.copyMakeBorder(image, padding[1], padding[3], padding[0], padding[2],  cv2.BORDER_CONSTANT, value=padding_value)
    return image


def convert_hls_image(image_data): 
    #resize image 
    w,h = image_data.shape[1], image_data.shape[0]
    new_height = int(CALIBRATION_WIDTH*h/w)
    I = cv2.resize(image_data, (CALIBRATION_WIDTH, new_height))#BRG mode
    hls = cv2.cvtColor(I, cv2.COLOR_BGR2HLS)#cv2.COLOR_RGB2HLS)
    lower = np.uint8([0, 150, 0])
    upper = np.uint8([255, 255, 255])
    mask = cv2.inRange(hls, lower, upper)
    Threshold1 = 150
    Threshold2 = 350
    FilterSize = 5
    E = cv2.Canny(mask, Threshold1, Threshold2, FilterSize)
    lines = cv2.HoughLinesP(E, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=20)
    N = lines.shape[0]
    for i in range(N):
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]
       # cv2.line(I, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.line(I, (x1, y1), (x2, y2), (0, 0, 255), 2)
    #scale-back to original image 
    I = cv2.resize(I, (w,h))
    return I#, scale

def convert_perspective_image_old(image_data, froll, fpitch, fyaw, cameraParam, canvas_size=400): 
    newsize = (1920, 1080)
    #old_w, old_h = image_data.shape[1], image_data.shape[0]
    cameraParam.setMatrix_R(froll, fpitch, fyaw)
    cameraParam.calMatrix_P()
    trsnsMat=cameraParam.Mat_P[:, (0, 1, 3)]
    gray = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, newsize)
    
    if 1:#for debug
        #trsnsMat=[[1,0.2,2] ,[0.2,1,0], [0,0,1]]
        trsnsMat=[[6.81183210e+02,  5.35969903e+02, -4.00813033e+06],[-6.89487113e+01, -1.01957980e+02, -4.23842045e+06],[8.50933143e-02,  6.46864574e-01, -3.97867346e+03]]
        trsnsMat=np.asarray(trsnsMat, dtype=np.float64)
    Umap = np.zeros((5000, 5000))
    for i in range(200, 1900, 2):
        for j in range(100, 1080, 2):
            temp = normTransMat(np.dot(np.linalg.inv(trsnsMat), np.array([i, j, 1]).reshape((3, 1))))
            temp = temp / 10
            temp[0] = temp[0]
            y = int(np.round(temp[1]) + 2000)
            x = int(np.round(temp[0]) + 2000)

            if (x<0) or (x>5000) or (y<0) or (y>5000): 
                continue
            Umap[y, x] = gray[j, i]
    #gray2 = cv2.imresize(Umap, (old_w, old_h))
    gray2 = cv2.resize(Umap, (canvas_size,canvas_size)) #canvas size in main.py
    pil_image = Image.fromarray(gray2)
    #xinxin image_tk = ImageTk.PhotoImage(pil_image)
    image_tk = gray2 
    return image_tk, cameraParam

def convert_perspective_image(image_data, froll, fpitch, fyaw, cameraParam, canvas_size=400): 
    newsize = (1920, 1080)
    #old_w, old_h = image_data.shape[1], image_data.shape[0]
    cameraParam.setMatrix_R(froll, fpitch, fyaw)
    cameraParam.calMatrix_P()
    trsnsMat=cameraParam.Mat_P[:, (0, 1, 3)]
    Mat_R=cameraParam.Mat_R * 10
    print("Mat_R is", Mat_R, image_data.shape[::-1][1:])

    image_perspective = cv2.warpPerspective(image_data, Mat_R, image_data.shape[::-1][1:], flags=cv2.INTER_LINEAR)

    #xinxin image_tk = ImageTk.PhotoImage(pil_image)
    image_tk = image_perspective 
    return image_tk, cameraParam


def convert_road_enhance_image(image_data): 
    old_w, old_h = image_data.shape[1], image_data.shape[0]
    sigma_list = [15, 80, 250]
    new_height = int(CALIBRATION_WIDTH*old_h/old_w)
    resized_im = cv2.resize(image_data, (CALIBRATION_WIDTH, new_height))#BRG mode
    pil_im = Image.fromarray(cv2.cvtColor(resized_im, cv2.COLOR_BGR2RGB))
    img_amsrcr = retinex.automatedMSRCR(pil_im, sigma_list)
    img_amsrcr[np.where((img_amsrcr!=[255,255,255]).all(axis=2))]=[128,128,128]
    cv_image = np.array(img_amsrcr) 
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    cv_image = cv2.resize(cv_image, (old_w, old_h))
    return cv_image


def get_pixel_coordinate(canvas_x, canvas_y, scale, padding):
    #get pixel coordinate from canvas coordinate
    top_pad, left_pad = padding[1],padding[0]

    pixel_x = canvas_x-left_pad
    pixel_y = canvas_y - top_pad
    pixel_x = round(pixel_x/scale)
    pixel_y = round(pixel_y/scale)
    return pixel_x, pixel_y
def get_canvas_coordinate(pixel_x, pixel_y, scale, padding): 
    top_pad, left_pad = padding[1],padding[0]
    canvas_x = round(pixel_x * scale)
    canvas_y = round(pixel_y*scale)

    canvas_x = canvas_x + left_pad
    canvas_y = canvas_y + top_pad
    return canvas_x, canvas_y

def convert_canvas_to_pixel(canvas_pt_list, scale, pad): 
    pixels = copy.deepcopy(canvas_pt_list)
    for i, pt_groups in enumerate(canvas_pt_list): 
        num_pt = len(pt_groups)//2
        for j in range(num_pt): 
            canvas_x, canvas_y = pt_groups[2*j], pt_groups[2*j+1]
            pixel_x, pixel_y = get_pixel_coordinate(canvas_x, canvas_y, scale, pad)
            pixels[i][2*j] = pixel_x
            pixels[i][2*j+1] = pixel_y
    return pixels
    
def convert_pixel_to_canvas(pixel_pt_list, scale, pad): 
    canvass = copy.deepcopy(pixel_pt_list)
    for i, pt_groups in enumerate(pixel_pt_list): 
        num_pt = len(pt_groups)//2
        for j in range(num_pt): 
            pixel_x, pixel_y = pt_groups[2*j], pt_groups[2*j+1]
            canvas_x, canvas_y = get_canvas_coordinate(pixel_x, pixel_y, scale, pad)
            canvass[i][2*j] = canvas_x
            canvass[i][2*j+1] = canvas_y
    return canvass    
'''
def resize_image_calibration(image_data): 
    w, h = image_data.shape[1], image_data.shape[0]
    if w >= h:
        baseW = 800
        wpercent = (baseW / float(w))
        hsize = int((float(h) * float(wpercent)))
        im_scaled = cv2.resize(image_data, (baseW, hsize))
    else:
        baseH = 450
        wpercent = (baseH / float(h))
        wsize = int((float(w) * float(wpercent)))
        im_scaled = cv2.resize(image_data, (wsize, baseH))
    scale = np.ones((2,1)) 
    new_w, new_h = im_scaled.shape[1], im_scaled.shape[0]
    scale[0] = w/(float)(new_w)
    scale[1] = h/(float)(new_h)
    return im_scaled, scale 

'''
