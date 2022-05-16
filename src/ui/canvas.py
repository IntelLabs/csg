#support zoom in/out, 
# https://stackoverflow.com/questions/41656176/tkinter-canvas-zoom-move-pan
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import math
import warnings
import cv2
import hashlib

MAX_ZOOM_RATE = 20 #20x
MAX_IMAGE_PIXELS = 1500000000  # maximum pixels in the image, use it carefully

class AutoScrollbar(ttk.Scrollbar):
    ''' A scrollbar that hides itself if it's not needed.
        Works only if you use the grid geometry manager '''
    def set(self, lo, hi):
        if float(lo) <= 0.0 and float(hi) >= 1.0:
            # grid_remove is currently missing from Tkinter!
            self.tk.call("grid", "remove", self)
        else:
            self.grid()
        ttk.Scrollbar.set(self, lo, hi)

    def pack(self, **kw):
        raise tk.TclError('Cannot use pack with this widget')

    def place(self, **kw):
        raise tk.TclError('Cannot use place with this widget')

class ZoomFrame(ttk.Frame):
    ''' Simple zoom with mouse wheel '''
    def __init__(self, mainframe, **opt):
        ''' Initialize the main Frame '''
        self.frameWidth = opt['width']
        self.frameHeight = opt['height']
        self.bg = opt['background']
        self.zoomable = False #default
        ttk.Frame.__init__(self, master=mainframe, width=self.frameWidth, height=self.frameHeight)
        self.init_layout()
        self.roi = []

    def init_layout(self): 
        # Vertical and horizontal scrollbars for canvas
        self.vbar = AutoScrollbar(self.master, orient='vertical')
        self.hbar = AutoScrollbar(self.master, orient='horizontal')
        self.vbar.grid(row=0, column=1, sticky='ns')
        self.hbar.grid(row=1, column=0, sticky='we')
        # Create canvas and put image on it
        self.canvas = tk.Canvas(self.master, highlightthickness=0,
                                width = self.frameWidth, height = self.frameHeight,
                                xscrollcommand=self.hbar.set, yscrollcommand=self.vbar.set,
                                background=self.bg)
        self.container = None
        self.canvas.grid(row=0, column=0, sticky='nswe')
        self.vbar.configure(command=self.scroll_y)  # bind scrollbars to the canvas
        self.hbar.configure(command=self.scroll_x)
        self.origin = self.canvas.xview()[0], self.canvas.yview()[0]
        # Make the canvas expandable
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)
        # Bind events to the Canvas
        self.canvas.bind('<Configure>', self.show_image)  # canvas is resized
        self.canvas.bind('<MouseWheel>', self.wheel)  # with Windows and MacOS, but not Linux
        self.canvas.bind('<Button-5>',   self.wheel)  # only with Linux, wheel scroll down
        self.canvas.bind('<Button-4>',   self.wheel)  # only with Linux, wheel scroll up
        self.image = None

    def load_image(self, image_pil): 
        self.image_raw  = image_pil
        self.image = image_pil
        self.width, self.height = self.image.size
        self.container = self.canvas.create_rectangle(0, 0, self.width, self.height, width=0)
        self.imscale = 1.0
        self.delta = 0.75
        self.show_image()

    def scroll_y(self, *args, **kwargs):
        ''' Scroll canvas vertically and redraw the image '''
        self.canvas.yview(*args, **kwargs)  # scroll vertically
        self.show_image()  # redraw the image

    def scroll_x(self, *args, **kwargs):
        ''' Scroll canvas horizontally and redraw the image '''
        self.canvas.xview(*args, **kwargs)  # scroll horizontally
        self.show_image()  # redraw the image

    def move_from(self, event):
        ''' Remember previous coordinates for scrolling with the mouse '''
        self.canvas.scan_mark(event.x, event.y)

    def move_to(self, event):
        ''' Drag (move) canvas to the new position '''
        self.canvas.scan_dragto(event.x, event.y, gain=1)
        self.show_image()  
    
    def mouse_click(self, event): 
        print('Canvas level: mouse click')
        pass
         

    def wheel(self, event):
        ''' Zoom with mouse wheel '''
        if not self.zoomable: 
            return
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        if self.container is None: #no image load return
            return

        bbox = self.canvas.bbox(self.container)  # get image area
        if bbox[0] < x < bbox[2] and bbox[1] < y < bbox[3]: pass  # Ok! Inside the image
        else: return  # zoom only inside image area
        scale = 1.0
        # Respond to Linux (event.num) or Windows (event.delta) wheel event
        if event.num == 5 or event.delta == -120:  # scroll down
            i = min(self.width, self.height)
            #if int(i * self.imscale) < 30: return  # image is less than 30 pixels
            new_scale = self.imscale / self.delta
            if new_scale>MAX_ZOOM_RATE:  return
            self.imscale /= self.delta
            scale        /= self.delta
        if event.num == 4 or event.delta == 120:  # scroll up
            i = min(self.canvas.winfo_width(), self.canvas.winfo_height())
            if i < self.imscale: return  # 1 pixel is bigger than the visible area
            new_scale = self.imscale*self.delta
            if new_scale <1.0: return #disable smaller than the original image
            self.imscale *= self.delta
            scale        *= self.delta
        self.canvas.scale('all', x, y, scale, scale)  # rescale all canvas objects
        self.show_image()


    def show_image(self, event=None):
        ''' Show image on the Canvas '''
        if self.container is None: 
            return 
        bbox1 = self.canvas.coords(self.container)  # get image area
        # Remove 1 pixel shift at the sides of the bbox1
        bbox1 = (bbox1[0] + 1, bbox1[1] + 1, bbox1[2] - 1, bbox1[3] - 1)
        bbox2 = (self.canvas.canvasx(0),  # get visible area of the canvas
                 self.canvas.canvasy(0),
                 self.canvas.canvasx(self.canvas.winfo_width()),
                 self.canvas.canvasy(self.canvas.winfo_height()))

        bbox = [min(bbox1[0], bbox2[0]), min(bbox1[1], bbox2[1]),  # get scroll region box
                max(bbox1[2], bbox2[2]), max(bbox1[3], bbox2[3])]
        if bbox[0] == bbox2[0] and bbox[2] == bbox2[2]:  # whole image in the visible area
            bbox[0] = bbox1[0]
            bbox[2] = bbox1[2]
        if bbox[1] == bbox2[1] and bbox[3] == bbox2[3]:  # whole image in the visible area
            bbox[1] = bbox1[1]
            bbox[3] = bbox1[3]
        self.canvas.configure(scrollregion=bbox)  # set scroll region
        x1 = max(bbox2[0] - bbox1[0], 0)  # get coordinates (x1,y1,x2,y2) of the image tile
        y1 = max(bbox2[1] - bbox1[1], 0)
        x2 = min(bbox2[2], bbox1[2]) - bbox1[0]
        y2 = min(bbox2[3], bbox1[3]) - bbox1[1]
        if int(x2 - x1) > 0 and int(y2 - y1) > 0:  # show image if it in the visible area
            x = min(int(x2 / self.imscale), self.width)   # sometimes it is larger on 1 pixel...
            y = min(int(y2 / self.imscale), self.height)  # ...and sometimes not
            image = self.image.crop((int(x1 / self.imscale), int(y1 / self.imscale), x, y))
            #image.save('test.jpg')
            self.roi = [int(x1 / self.imscale), int(y1 / self.imscale), x, y, 
                        int(x2-x1), int(y2-y1),
                        max(bbox2[0], bbox1[0]), max(bbox2[1], bbox1[1])]
            imagetk = ImageTk.PhotoImage(image.resize((int(x2 - x1), int(y2 - y1))))
            imageid = self.canvas.create_image(max(bbox2[0], bbox1[0]), max(bbox2[1], bbox1[1]),
                                               anchor='nw', image=imagetk)
            self.canvas.lower(imageid)  # set image into background
            self.canvas.imagetk = imagetk  # keep an extra reference to prevent garbage-collection

    def reset_canvas(self): 
        '''scale=1, fit to canvas'''
        bbox = self.canvas.coords(self.container)
        if bbox[0]==0 and bbox[1]==0 and bbox[2]== self.frameWidth and bbox[3]==self.frameHeight: 
            return
        #TODO: any solution instead of delete first
        for children in self.master.winfo_children():
            children.destroy()

        self.init_layout()
        self.load_image(self.image_raw)

    def set_zoomable(self, zoomable=False): 
        self.zoomable = zoomable
        if (not self.zoomable) and (not self.image is None): 
            self.reset_canvas()
        
    def get_pixel_values(self, points): 
        #convert canvas to pixel 
        pt_pixel = []
        for pt in points: 
            x =(pt[0] - self.roi[-2])/self.imscale +self.roi[0]
            y = (pt[1]-self.roi[-1])/self.imscale + self.roi[1]
            pt_pixel.append([x,y])
        #drw on raw image and viz
       # from PIL import ImageDraw
       # draw = ImageDraw.Draw(self.image_raw)
       # print('pixel: ', pt_pixel)
       # for pt in pt_pixel: 
       #     x,y = pt[0], pt[1]
       #     r = 5
       #     leftUpPoint = (x-r, y-r)
       #     rightDownPoint = (x+r, y+r)
       #     draw.ellipse([leftUpPoint, rightDownPoint], fill=(255,0,0,255))
       # self.image_raw.save('pixel.jpg')

        return pt_pixel
