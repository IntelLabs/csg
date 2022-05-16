import random
import numpy as np
import secrets
#ref: http://bootflat.github.io/color-picker.html
hex_colors = ['#edbc6d','#dd828e','#bada55','#588585','#e39a31','#f48670','#9dffb0',
              '#e8c15f','#c90579','#eb491e','#9a6d41','#7cb07c','#91a6b4','#f8b6a8',
              '#008080','#548b54']
marker_colors = {"Vanishing Point":"red", "Parallel Line":"red", "Ref Point":"yellow", "Ref Line":"yellow",
                "East":"red", "North":"blue",  "West":"yellow", "South":"green"}
def rand_rgb(): 
    secretGenerator = secrets.SystemRandom()
    r = secretGenerator.randint(0,255)
    g = secretGenerator.randint(0,255)
    b = secretGenerator.randint(0,255)
    #r = random.randint(0,255)
    #g = random.randint(0, 255)
    #b = random.randint(0, 255)
    return r,g,b

def rand_hex(): 
    de, re, we = rand_rgb()
    return rgb_to_hex(de, re, we)

def rgb_to_hex(de, re, we): 
    return "#%02x%02x%02x"%(de, re,we)

def hex_to_rgb(hex): 
    hex = hex.lstrip('#')
    return tuple(int(hex[i:i+2], 16) for i in (0, 2, 4))

def norm_rgb(arr): 
    for i in range(3): 
        arr[i] = min(255, max(0, arr[i]))
    return arr  

def alpha_hex(hex, alpha=0.6): 
    rgb = hex_to_rgb(hex)
    rgb = (np.asarray(rgb)*alpha+(1-alpha)*255).astype(int)
    rgb = norm_rgb(rgb)
    return rgb_to_hex(rgb[0], rgb[1], rgb[2])

def scale_hex(hex, scale): 
    rgb = hex_to_rgb(hex)
    rgb = (np.asarray(rgb)* scale).astype(int)
    rgb = norm_rgb(rgb)
    return rgb_to_hex(rgb[0], rgb[1], rgb[2])

def get_hex_by_index(index): 
    color_num = len(hex_colors)
    index = max(0, index)
    index = min(color_num-1, index)
    return hex_colors[index]

def get_finegrained_color(tag, index): 
    if tag=='red': 
        lst = ['#FF0000', '#CF000F','#D24D57','#DB5A6B','#C93756','#DC3023']
    elif tag=='yellow': 
        lst = ['#FFFF00','#D9B611','#F5D76E','#FFB61E','#FFA400','#E29C45']

    return lst[index%len(lst)]