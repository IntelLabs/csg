#frame to set lane number in different directions 
from tkinter import *
from tkinter import ttk
from PIL import Image, ImageTk
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

class LaneSettingFrame(ttk.Frame):
    def __init__(self, mainframe, image_bgr, input_lane_num=dict(), **opt):
        #current only support square frame: width=height
        self.parent = mainframe
        self.frameWidth = opt['width'] #only test on 400x400, 
        self.frameHeight = opt['height']
        self.canvasWidth=self.frameWidth
        self.canvasHeight = self.frameHeight
        self.defaultLaneNum = 1

        self.possibleLaneNumber =[i for i in range(0, 6)]
      #  self.defaultCbIndex = self.possibleLaneNumber.index(self.defaultLaneNum)
        self.gridsize =self.canvasWidth/6 #6x6 grids

        ttk.Frame.__init__(self, master=mainframe,width=self.frameWidth,height=self.frameHeight,
                            relief=SUNKEN,borderwidth=2)
        #self.pack(fill=BOTH, expand=1)

        image_bgr = cv2.resize(image_bgr, (self.frameWidth, self.frameHeight))
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        imagetk = ImageTk.PhotoImage(Image.fromarray(rgb))#file='test.gif')

        self.canvas = Canvas(self, width=self.canvasWidth, height=self.canvasHeight,background='white')
        self.canvas.pack()
        
        self.tkimage = imagetk #must save, 
        self.canvas.create_image(int(self.canvasWidth/2),int(self.canvasHeight/2),image=imagetk)#,anchor=NW)#defualt anchor=center
                #init 
        all_orientations = ['ne', 'nw', 'wn', 'ws', 'sw', 'se','es','en']
        self.laneNum = dict()
        for o in all_orientations:
            if  bool(input_lane_num): 
                self.laneNum[o] = input_lane_num[o]
            else: 
                self.laneNum[o] = self.defaultLaneNum
            self.redraw_lane(o)
            
        combobox_width = 5
        self.neCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)#north-east
        self.neCb.current(self.possibleLaneNumber.index(self.laneNum['ne']))
        self.canvas.create_window(int(self.gridsize*4)+combobox_width*2,10, anchor=NW, window=self.neCb)
        self.nwCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.nwCb.current(self.possibleLaneNumber.index(self.laneNum['nw']))
        self.canvas.create_window(int(self.gridsize),10, anchor=NW, window=self.nwCb)
        self.wnCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.wnCb.current(self.possibleLaneNumber.index(self.laneNum['wn']))
        self.canvas.create_window(10, int(self.gridsize*2)-4*combobox_width, anchor=NW, window=self.wnCb)
        self.wsCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.wsCb.current(self.possibleLaneNumber.index(self.laneNum['ws']))   
        self.canvas.create_window(10, self.canvasHeight-int(self.gridsize*2), anchor=NW, window=self.wsCb)
        self.swCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.swCb.current(self.possibleLaneNumber.index(self.laneNum['sw']))
        self.canvas.create_window(int(self.gridsize),self.canvasHeight-30, anchor=NW, window=self.swCb)
        self.seCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.seCb.current(self.possibleLaneNumber.index(self.laneNum['se'])) 
        self.canvas.create_window(int(self.gridsize*4.5),self.canvasHeight-30, anchor=NW, window=self.seCb)
        self.esCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.esCb.current(self.possibleLaneNumber.index(self.laneNum['es']))
        self.canvas.create_window(self.canvasWidth-int(self.gridsize),self.canvasHeight-int(self.gridsize*2), anchor=NW, window=self.esCb)
        self.enCb = ttk.Combobox(self, state='readonly',width=combobox_width,values=self.possibleLaneNumber)
        self.enCb.current(self.possibleLaneNumber.index(self.laneNum['en']))
        self.canvas.create_window(self.canvasWidth-int(self.gridsize),int(self.gridsize*2)-4*combobox_width, anchor=NW, window=self.enCb)

       # self.canvas.create_oval(200-3,200-3,200+3,200+3,fill='black')
       # self.canvas.create_oval(400-3,400-3,400+3,400+3,fill='black')

        self.neCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "ne"))
        self.nwCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "nw"))
        self.wnCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "wn"))
        self.wsCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "ws"))
        self.swCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "sw"))
        self.seCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "se"))
        self.esCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "es"))
        self.enCb.bind('<<ComboboxSelected>>', lambda event:self.select(event, "en"))
       # self.canvas.bind("<Button-1>", self.mouse_click)

    def mouse_click(self, event): #debugging
        print("event: ({}, {})".format(event.x, event.y))
        print("canvas:({}, {})".format(self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)))
    
    def set_lane_num(self, input_lane_num): 
        if input_lane_num: 
            self.laneNum = input_lane_num
        self.set_cb_vlaue()
        for key in self.laneNum: 
            self.redraw_lane(key)
            
    def select(self, event, tag):
        orien = tag
        value = int(event.widget.get())
        old_value = self.laneNum[orien]
        if not value==old_value: 
            self.laneNum[orien] = value
            self.redraw_lane(orien)

    def redraw_lane(self, orientation): 
        #redraw specific directions
        draw_lane_num = self.laneNum[orientation]
        dash_mode = (45,20)
        self.canvas.delete(orientation+'_lines')#clean lines for redraw
        if draw_lane_num ==1: #one lane, no need to draw
            return
        if orientation =='nw':
            startx,endx = self.gridsize*2, self.gridsize*3
            starty, endy = 0, self.gridsize*2
            stepx = (endx-startx)/(draw_lane_num+1)
            draw_dir = 'V'#vertical 
        elif orientation =='ne': 
            startx,endx = self.gridsize*3, self.gridsize*4
            starty, endy = 0, self.gridsize*2
            stepx = (endx-startx)/(draw_lane_num+1) 
            draw_dir = 'V'
        elif orientation =='wn': 
            startx, endx = 0, self.gridsize*2
            starty, endy = self.gridsize*2, self.gridsize*3
            stepy = (endy-starty)/(draw_lane_num+1)
            draw_dir ='H'#horizontal
        elif orientation =='ws':
            startx, endx = 0, self.gridsize*2
            starty, endy = self.gridsize*3, self.gridsize*4
            stepy = (endy-starty)/(draw_lane_num+1)             
            draw_dir ='H'
        elif orientation =='sw':
            startx,endx = self.gridsize*2, self.gridsize*3
            starty, endy = self.frameHeight-self.gridsize*2, self.frameHeight-0
            stepx = (endx-startx)/(draw_lane_num+1)            
            draw_dir ='V'
        elif orientation =='se': 
            startx,endx = self.gridsize*3, self.gridsize*4
            starty, endy = self.frameHeight-self.gridsize*2, self.frameHeight-0
            stepx = (endx-startx)/(draw_lane_num+1)   
            draw_dir ='V'
        elif orientation =='es':
            startx, endx = self.frameWidth- self.gridsize*2, self.frameWidth
            starty, endy = self.gridsize*3, self.gridsize*4
            stepy = (endy-starty)/(draw_lane_num+1)         
            draw_dir ='H'
        elif orientation =='en':
            startx, endx = self.frameWidth- self.gridsize*2, self.frameWidth
            starty, endy = self.gridsize*2, self.gridsize*3
            stepy = (endy-starty)/(draw_lane_num+1)               
            draw_dir ='H'
        
        if draw_lane_num ==0: 
            self.canvas.create_rectangle(startx, starty, endx, endy, outline='white',fill='white', width=2, tag=orientation+'_lines') 
        else: 
            if draw_dir =='V': 
                for i in range(draw_lane_num):
                    x = startx+stepx*(i+1)
                    self.canvas.create_line(x, starty, x, endy, fill='white', dash=dash_mode, width=2, tag=orientation+'_lines') 
            else: 
                for i in range(draw_lane_num):
                    y = starty+stepy*(i+1)
                    self.canvas.create_line(startx, y, endx, y, fill='white', dash=dash_mode, width=2, tag=orientation+'_lines') 
    
    def set_cb_vlaue(self): 
        #set with self.laneNum
        for o in self.laneNum: 
            if o == 'ne': 
                self.neCb.current(self.possibleLaneNumber.index(self.laneNum['ne']))
            if o =='nw': 
                self.nwCb.current(self.possibleLaneNumber.index(self.laneNum['nw']))
            if o=='wn': 
                self.wnCb.current(self.possibleLaneNumber.index(self.laneNum['wn']))
            if o=='ws': 
                self.wsCb.current(self.possibleLaneNumber.index(self.laneNum['ws']))
            if o=='sw': 
                self.swCb.current(self.possibleLaneNumber.index(self.laneNum['sw']))
            if o=='se': 
                self.seCb.current(self.possibleLaneNumber.index(self.laneNum['se']))
            if o=='es': 
                self.esCb.current(self.possibleLaneNumber.index(self.laneNum['es']))
            if o=='en': 
                self.enCb.current(self.possibleLaneNumber.index(self.laneNum['en']))


def start(root, image_cv):
    if(1): 
        #checking iamges
        width,height = image_cv.shape[1], image_cv.shape[0]
        gridsize = width/6
        startx,endx = gridsize*2, gridsize*3
        starty, endy = 0, int(gridsize*2)
        draw_lane_num = 3
        stepx = (endx-startx)/(draw_lane_num+1)
        for i in range(draw_lane_num):
            x = int(startx+stepx*(i+1))
            cv2.line(image_cv, (x,starty), (x, endy),(255,255,255))
        cv2.imshow("", image_cv)
        cv2.waitKey(-1)


    startframe = Frame(root)
    canvas = Canvas(startframe,width=1280,height=720)

    startframe.pack()
    canvas.pack()

    # Escape / raw string literal
    one = PhotoImage(file='test.gif')
    root.one = one  # to prevent the image garbage collected.
    canvas.create_image(400,400,image=one, anchor='nw')

if __name__ == "__main__":
    root = Tk()
    image_cv = cv2.imread('/home/lidanzha/hdd/work/CSG/critical-scenario-generation_tester/src/ui/test.jpg')
    LaneSettingFrame(root, image_cv, width=400, height=400)
    root.mainloop()

    #start(root, image_cv)
