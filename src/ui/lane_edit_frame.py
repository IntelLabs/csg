
#frame to set lane number in different directions 
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import itertools
from PIL import Image, ImageTk
import numpy as np
#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
#import sys
#sys.path.append('/home/lidanzha/hdd/work/CSG/critical-scenario-generation/')
import cv2
from src.ui.image_utils import fit_image_to_canvas
from src.ui.drag_line import DragPolygon
from src.ui.image_utils import convert_canvas_to_pixel, convert_pixel_to_canvas
from src.convert_util import project_2d_to_3d_depth, project_2d_to_3d_depth_arr
from src.convert_util import get_abs_pose, project_3d_to_2d
from src.road.lane import inRect, resample_points


class View_AskLaneID(tk.Toplevel):
    def __init__(self, master, value=u""):
        self.value = None
        tk.Toplevel.__init__(self, master)
        self.protocol("WM_DELETE_WINDOW",self.destroy)
        self.attributes('-topmost',True)
        self.transient()
        self.resizable(False, False)
        top = self.top = self
        x = master.winfo_x()
        y = master.winfo_y()
        
        self.geometry("+%d+%d" % (x + 400, y + 200))
        self.l = ttk.Label(top, text=u"Lane ID:")
        self.l.grid(row=0, column=0, sticky='w')
        #self.l.pack()
        self.e = ttk.Entry(top)
        self.e.grid(row=1, column=0, sticky='ws')
        #self.e.pack()
        self.b = ttk.Button(top, text='Ok', command=self.save)
        self.b.grid(row=2, column=0)
       # self.b.pack()
        self.c = ttk.Button(top, text='Cancel', command=self.cancel)
        self.c.grid(row=2, column=1)
       # self.c.pack

        if value: self.e.insert(-1, value)
        self.e.focus_set()
        top.bind('<Return>', self.save)

    def save(self, *_):
        self.value = self.e.get()
        try: 
            val = int(self.value)
        except: 
            messagebox.showinfo(title='Warning', message='ID should be a number')
            self.e.focus_set()
            return 
        if val <0: 
            return
        self.destroy()

    def cancel(self, *_): 
        self.value = -1
        self.destroy()


class LaneEditFrame(tk.Toplevel):
    def __init__(self, master, video_capture, init_lane2d, param, winsize=20,width=1200, height=600):
        top=self.top=self
        self.frameWidth = width
        self.frameHeight = height
        self.master = master
        tk.Toplevel.__init__(self, master=master,
                          width=self.frameWidth,height=self.frameHeight,
                          borderwidth=2)
        #x = master.winfo_x()
        #y = master.winfo_y()
        #self.geometry('{}x{}+{}+{}'.format(width,height, x+200,y+50))
        self.geometry('{}x{}'.format(width,height))
        self.protocol("WM_DELETE_WINDOW",self.close)
        #self.attributes('-topmost',True)
        self.transient()
        #self.resizable(False, False)
        self.top.title("Lane Labeling Tool")
        self.videoCap = video_capture
        
        self.toolWidth = 200 #space left on the right
        self.toolHeight = 100#space left at the bottom
        self.canvasWidth=self.frameWidth-self.toolWidth
        self.canvasHeight = self.frameHeight - self.toolHeight
        self.frameId = 0
        self.frameNum = int(self.videoCap.get(7))

        self.canvasFrame = ttk.Frame(self)
        self.toolFrame = ttk.Frame(self)
        self.statusFrame = ttk.Frame(self)
        self.canvasFrame.grid(row=0, column=0, sticky="n")
        self.toolFrame.grid(row=0, column=1, sticky="nsew")
        self.statusFrame.grid(row=1, column=0, columnspan=2, sticky="ew")

        
        self.canvasDisplayFrame = ttk.Frame(self.canvasFrame)
        self.videoNavigationFrame = ttk.Frame(self.canvasFrame)
        self.canvasDisplayFrame.grid(row=0, column=0,padx=5, sticky="w")
        self.videoNavigationFrame.grid(row=1, column=0,padx=5, sticky="w")
        self.canvasPaintFrame = tk.Canvas(self.canvasDisplayFrame, highlightthickness=0,
                                width = self.canvasWidth, height = self.canvasHeight,
                                background="#90AFC5")
        self.canvasPaintFrame.pack(side=tk.TOP)
        self.frameLabel = tk.Label(self.videoNavigationFrame, text="Frame ")
        self.gotoFirstBtn = tk.Button(self.videoNavigationFrame, text="|<", command=self.goto_first)
        self.gotoPrev10Btn = tk.Button(self.videoNavigationFrame, text="<<", command=self.goto_prev10)
        self.gotoPrevBtn = tk.Button(self.videoNavigationFrame, text="<", command=self.goto_prev)
        #self.playBtn = tk.Button(self.videoNavigationFrame,text="|>", command=self.play)
        self.gotoNextBtn = tk.Button(self.videoNavigationFrame, text=">", command=self.goto_next)
        self.gotoNext10Btn = tk.Button(self.videoNavigationFrame, text=">>", command=self.goto_next10)
        self.gotoLastBtn = tk.Button(self.videoNavigationFrame, text=">|", command=self.goto_last)
		
        self.frameLabel.pack(fill=tk.BOTH, side=tk.TOP)
        self.frameLabel.config(font=("Helvetica", 16))#Helvetica, Rouge
        self.gotoFirstBtn.pack(fill=tk.X, side=tk.LEFT, padx=1, expand=0)
        self.gotoPrev10Btn.pack(fill=tk.X, side=tk.LEFT, padx=1, expand=0)
        self.gotoPrevBtn.pack(fill=tk.X, side=tk.LEFT, padx=1, expand=0)
        #self.playBtn.pack(fill=tk.X, side=LEFT, padx=1, expand=0)
        self.gotoNextBtn.pack(fill=tk.X, side=tk.LEFT, padx=1, expand=0)
        self.gotoNext10Btn.pack(fill=tk.X, side=tk.LEFT, padx=1, expand=0)
        self.gotoLastBtn.pack(fill=tk.X, side=tk.LEFT, padx=1, expand=0)
		
        
        self.controlFrame = ttk.Frame(self.toolFrame)
        self.controlFrame.pack(fill=tk.X, padx=10, side=tk.TOP)
        self.EditBtn =tk.Button(self.controlFrame, text="Edit", command=self.edit,disabledforeground='red')
        self.EditBtn.pack(fill=tk.X, pady=10, side=tk.TOP)
        self.AddBtn = tk.Button(self.controlFrame, text="Add", command=self.insert,disabledforeground='red')
        self.AddBtn.pack(fill=tk.X, side=tk.TOP)
        self.InterpBtn = tk.Button(self.controlFrame, text="Interpolation", command=self.interp)
        self.InterpBtn.pack(fill=tk.X, side=tk.TOP)
        self.CloseBtn = tk.Button(self.controlFrame, text="Close", command=self.close)
        self.CloseBtn.pack(fill=tk.X, side=tk.TOP)
        ttk.Separator(self.controlFrame, orient=tk.HORIZONTAL).pack(fill=tk.BOTH, pady=10, side=tk.TOP)
        ttk.Label(self.controlFrame, text='Lane List').pack(fill=tk.X, side=tk.TOP)
        self.LaneListBox = tk.Listbox(self.controlFrame)
        self.LaneListBox.pack(fill=tk.X, side=tk.TOP)
        self.LaneListEditFrame = ttk.Frame(self.controlFrame)
        self.LaneListEditFrame.pack(fill=tk.X, side=tk.TOP)
        self.DeleteBtn = tk.Button(self.LaneListEditFrame, text="Delete", command=self.delete)
        self.ModifyBtn = tk.Button(self.LaneListEditFrame, text="Modify ID", command=self.modify)
        self.DeleteBtn.grid(row=0, column=0)
        self.ModifyBtn.grid(row=0, column=1)
        
        self.statusLabel = tk.Label(self.statusFrame, text="Status:", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.statusLabel.pack(fill=tk.X, side=tk.LEFT, expand=True)

        self.canvasPaintFrame.bind("<Button-1>", self.mouse_click)
        self.canvasPaintFrame.bind("<Motion>", self.mouse_move)
        self.canvasPaintFrame.bind("<B1-Motion>", self.mouse_drag)
        self.canvasPaintFrame.bind("<ButtonRelease-1>", self.mouse_release)
        self.canvasPaintFrame.bind("<Double-Button-1>", self.mouse_double_click)

        self.LaneListBox.bind('<Key>', self.press_lane_list)
        self.LaneListBox.bind('<Double-Button>', self.double_click_lane_list)

        self.edit_mode = True
        self.lane_2d = init_lane2d
        self.depth_list = param['depth']
        self.intrisic = param['intrisic']
        self.pose_list = param['extrisic']
        self.num_frames = min(len(self.pose_list), len(self.depth_list))
        self.winsize = winsize
        self.temp_lane = dict()#temporary recording during drawing 
        self.lane_2d_canvas = dict()#used to record canvas coordinates 
        self.draw_line_objs= []
        self.interp_frame_num = 10
        self.goto()

    def goto(self): 
        self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, self.frameId)
        _, self.imgRaw = self.videoCap.read()
        self.imgDisp, _, self.imgScale, self.imgPadding = fit_image_to_canvas(self.imgRaw,
										desired_width=self.canvasWidth, 
                                        desired_height=self.canvasHeight)
        self.canvasPaintFrame.delete('all')
        rgb = cv2.cvtColor(self.imgDisp, cv2.COLOR_BGR2RGB)
        self.rgb = ImageTk.PhotoImage(image = Image.fromarray(rgb))
        self.canvasPaintFrame.create_image(0, 0, image = self.rgb, anchor = tk.NW)
        
        self.frameLabel.config(text ='Frame {}'.format(self.frameId+1))

        # convert pixel to canvas 
        self.lane_2d_canvas = dict()
        if self.frameId in self.lane_2d: 
            for _, (lane_id, lane_pixels) in enumerate(self.lane_2d[self.frameId].items()):
                lane_canvas = [] 
                for px in lane_pixels: 
                    canvas_xy = convert_pixel_to_canvas([px], self.imgScale, self.imgPadding)[0]
                    lane_canvas.append(canvas_xy)
                self.lane_2d_canvas[lane_id] = lane_canvas
        self.refresh() #draw 
    
    def edit(self): 
        self.edit_mode = True
        self.refresh(draw_canvas=False, draw_list_table=False)

    def insert(self): 
        self.edit_mode = False
        self.refresh(draw_canvas=False, draw_list_table=False)
        #click several points for a line, ending with double-click

    def refresh(self, draw_canvas=True, draw_list_table=True, draw_button=True ): 
        #refresh canvas & lane list & button 
        #only used lane_2d_canvas, which is only recording current frame 
       
        if bool(self.lane_2d_canvas) and draw_canvas: 
            for obj in self.draw_line_objs: 
                obj.erase()
            self.draw_line_objs = []
            for _, (lid, pt_list) in enumerate(self.lane_2d_canvas.items()): 
                line = DragPolygon(self, self.canvasPaintFrame, pt_list, 
                                     lid, line_color='yellow', node_color='red',tag='lane',
                                     title=str(lid))
                self.draw_line_objs.append(line)
        
        if bool(self.lane_2d_canvas) and draw_list_table: 
            self.LaneListBox.delete(0, tk.END)
            for _, (lid, pt_list) in enumerate(self.lane_2d_canvas.items()): 
                txt = '{}:({},{})-({},{})'.format(lid, int(pt_list[0][0]), int(pt_list[0][1]), 
                                                int(pt_list[-1][0]), int(pt_list[-1][1]))
                self.LaneListBox.insert(tk.END, txt)	

        if draw_button: 
            if self.edit_mode: 
                self.EditBtn['state'] = 'disable'
                self.AddBtn['state'] = 'normal'
                self.EditBtn.configure(background='#6495ed')
                self.AddBtn.configure(background=self.cget('bg'))
                self.statusLabel.config(text = "Lane Edit mode")
            else: 
                self.EditBtn['state'] = 'normal'
                self.AddBtn['state'] = 'disable'
                self.EditBtn.configure(background=self.cget('bg'))#'#6495ed')
                self.AddBtn.configure(background='#6495ed')
                self.statusLabel.config(text = "Adding a lane")
    
    def update_line(self, target_object_id): 
        #update value 
        for line_obj in self.draw_line_objs: 
            if line_obj.objectID == target_object_id: 
                points = line_obj.points
                self.lane_2d_canvas[target_object_id]=points
        self.refresh(draw_canvas=False, draw_list_table=True, draw_button=False)
    
    #interpolate line in 3D space and reproject to 2D 
    def interp(self): 
        self.save_result() #convert canvas to pixel
        
        #step1: for each line, check if need to interp
        curr_lane_ids = list(self.lane_2d_canvas.keys())
        lane_num = len(curr_lane_ids)
        lost_lanes = [False]*lane_num
        last_frame_id = min(self.num_frames-1, self.frameId+self.interp_frame_num)
        for i in range(self.frameId+1, last_frame_id): 
            if i not in self.lane_2d.keys(): #no lane in this frame
                for j in range(lane_num): 
                    lost_lanes[j] = True
                break
            temp = list(self.lane_2d[i].keys())
            for j,k in enumerate(curr_lane_ids): 
                if k not in temp: 
                    lost_lanes[j] = True
        if not any(lost_lanes): #all lanes are exist in following frames, do nothing
            return 
        #we project to other frames, which is better than 3d poly fitting
        for j, lid in enumerate(curr_lane_ids): 
            if not lost_lanes[j]: #need to interpolation?
                continue
            #get the poly in 3D 
            raw_pt2d = self.lane_2d[self.frameId][lid]
            pt2d  = resample_points(raw_pt2d, max(raw_pt2d.shape[0], 10))
            depth = self.depth_list[self.frameId]
            ds = [depth[int(pt2d[k,1]), int(pt2d[k,0])] for k in range(pt2d.shape[0])]
            pt3d = project_2d_to_3d_depth_arr(np.transpose(pt2d),\
                    self.intrisic, self.pose_list[self.frameId], np.array(ds))
            pt3d = np.transpose(pt3d)
            #interpolation for next frames
            for f in range(self.frameId+1, last_frame_id): 
                if (f in self.lane_2d.keys()) and (lid in self.lane_2d[f].keys()): 
                    continue #already exist, no not interp???
                #get 3d bound 
                pose = self.pose_list[f]
                proj_pt2d = project_3d_to_2d(np.transpose(pt3d), self.intrisic, pose)
                proj_pt2d = np.transpose(proj_pt2d)
                proj_pt2d = resample_points(proj_pt2d, raw_pt2d.shape[0], [min(raw_pt2d[:,0]), max(raw_pt2d[:,0])]) 
                if f not in self.lane_2d.keys(): 
                    self.lane_2d[f] = dict()
                self.lane_2d[f][lid] = proj_pt2d
            

    def save_result(self): 
        #convert canvas to pixel and saved to pt2d 
        if bool(self.lane_2d_canvas): 
            lane_2d_temp = dict()
            for _, (lid, canvas_coords) in enumerate(self.lane_2d_canvas.items()): 
                num_pt = len(canvas_coords)
                lane_xys = np.zeros((num_pt,2))
                for i, canvas_xy in enumerate(canvas_coords):
                    px = convert_canvas_to_pixel([canvas_xy], self.imgScale, self.imgPadding)
                    lane_xys[i,0] = px[0][0]
                    lane_xys[i,1] = px[0][1]
                lane_2d_temp[lid] = lane_xys
            self.lane_2d[self.frameId] = lane_2d_temp 
        else: #clean-up result
            self.lane_2d[self.frameId] = dict()
    def goto_first(self):
        self.save_result() 
        self.frameId = 0 
        self.goto()
    def goto_last(self): 
        self.save_result() 
        self.frameId = self.frameNum-1
        self.goto()
    def goto_prev(self): 
        self.save_result() 
        self.frameId = max(0, self.frameId-1)
        self.goto()
    def goto_prev10(self): 
        self.save_result() 
        self.frameId = max(0, self.frameId-10)
        self.goto()
    def goto_next(self): 
        self.save_result() 
        self.frameId = min(self.frameNum-1, self.frameId+1)
        self.goto()
    def goto_next10(self): 
        self.save_result() 
        self.frameId = min(self.frameNum-1, self.frameId+10)
        self.goto()

    # mouse operation on canvas
    def mouse_click(self, event): 
        RADIUS = 5
        if self.edit_mode == False: #insert mode
            x = self.canvasPaintFrame.canvasx(event.x)
            y = self.canvasPaintFrame.canvasy(event.y)
            oid1 = self.canvasPaintFrame.create_oval(x - RADIUS, y - RADIUS,
                        x + RADIUS, y + RADIUS,fill='red', tags='temp_oval')
            oid2 = self.canvasPaintFrame.create_oval(x - RADIUS, y - RADIUS,
                        x + RADIUS, y + RADIUS,fill='red', tags='temp_oval')
            
            lid = self.canvasPaintFrame.create_line(x,y,x,y, width=2,
                                        fill='yellow',tags='temp_line')
            if 'xs'in self.temp_lane.keys(): 
                self.temp_lane['xs'].append(x)
                self.temp_lane['ys'].append(y)
                self.temp_lane['oval'].append(oid1)
            else: 
                self.temp_lane['xs'] =[x]
                self.temp_lane['ys']=[y]
                self.temp_lane['oval']=[oid1]

            self.temp_lane['curr_x'] = x
            self.temp_lane['curr_y'] = y
            self.temp_lane['curr_line'] = lid
            self.temp_lane['endpoint'] = oid2
        else: 
            pass 												 
				
    def mouse_move(self, event): 
        if self.edit_mode == False: #insert mode
            #fresh lane drawing 
            if not bool(self.temp_lane): 
                return 
            x = self.canvasPaintFrame.canvasx(event.x)
            y = self.canvasPaintFrame.canvasy(event.y)
            delta_x = event.x - self.temp_lane['curr_x']
            delta_y = event.y - self.temp_lane['curr_y']
            self.canvasPaintFrame.move(self.temp_lane["endpoint"], delta_x, delta_y)
            self.canvasPaintFrame.coords(self.temp_lane['curr_line'] , 
                                        self.temp_lane['xs'][-1], self.temp_lane['ys'][-1],
                                        x,y)
            self.temp_lane['curr_x'] = x
            self.temp_lane['curr_y'] = y
        else: #adjust point 
            pass 
    def mouse_drag(self, event): 
        if self.edit_mode == False: #insert mode
            if not bool(self.temp_lane): #empty?
                return
            #fresh lane drawing 
            x = self.canvasPaintFrame.canvasx(event.x)
            y = self.canvasPaintFrame.canvasy(event.y)
            delta_x = event.x - self.temp_lane['curr_x']
            delta_y = event.y - self.temp_lane['curr_y']
            self.canvasPaintFrame.move(self.temp_lane["endpoint"], delta_x, delta_y)
            self.canvasPaintFrame.coords(self.temp_lane['curr_line'] , 
                                        self.temp_lane['xs'][-1], self.temp_lane['ys'][-1],
                                        x,y)
            self.temp_lane['curr_x'] = x
            self.temp_lane['curr_y'] = y
        else: #adjust point 
            pass 
    def mouse_release(self, event): 
        pass

    def mouse_double_click(self, event): 
        #finish edition 
        self.edit_mode = True #auto switch to edit mode

        w = View_AskLaneID(self.master)  # canvasPaintFrame)
        self.master.wait_window(w.top)
        if int(w.value)>=0:
            #do real lane assignment and drawing  
            lid = int(w.value)
            temp_pts = [list(x) for x in zip(self.temp_lane['xs'], self.temp_lane['ys'])]

            self.lane_2d_canvas[lid] = temp_pts
            
        self.canvasPaintFrame.delete('temp_oval')
        self.canvasPaintFrame.delete('temp_line')
        self.temp_lane = dict()
        self.edit()
        self.refresh()
            
    def press_lane_list(self, event): 
        c = repr(event.char)
        if c == "'\\x7f'":	# delete press
            widget = event.widget
            selection = widget.curselection()
            if len(selection)>0:
                self.delete_item(selection[0])
			
    def double_click_lane_list(self, event):
        widget = event.widget
        selection = widget.curselection()
        if len(selection)>0: 
            self.modify_item(selection[0])
        
    def delete(self): 
        sel = self.LaneListBox.curselection()
        if len(sel)>0:
            self.delete_item(sel[0])
    
    def delete_item(self, sel): 
        result = messagebox.askquestion(
                "Delete", "Are You Sure?", icon='warning')
        if result == 'yes': 
            all_items = self.LaneListBox.get(0, tk.END)  
            txt = all_items[sel]
            ss = txt.split(':')
            lid = int(ss[0])
            del self.lane_2d_canvas[lid]
            self.save_result()
            self.canvasPaintFrame.delete('temp_oval')
            self.canvasPaintFrame.delete('temp_line')
            for obj in self.draw_line_objs: 
                obj.erase()
            self.LaneListBox.delete(0, tk.END)
            self.refresh(draw_button=False)   


    def modify(self): 
        sel = self.LaneListBox.curselection()
        if len(sel)>0:
            self.modify_item(sel[0]) 
    def modify_item(self, sel): 
        w = View_AskLaneID(self)
        self.master.wait_window(w.top)
        if int(w.value)>=0:
            all_items = self.LaneListBox.get(0, tk.END) 
            txt = all_items[sel]
            ss = txt.split(':')
            old_lid = int(ss[0])
            new_lid = int(w.value)
            points = self.lane_2d_canvas[old_lid]
            del self.lane_2d_canvas[old_lid]
            self.lane_2d_canvas[new_lid] = points
            self.save_result()
            self.refresh(draw_button=False)
    def close(self): 
        #pass parameter to main-ui
        self.save_result() 
        self.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    videoFilename = '/home/lidanzha/hdd/work/CSG/critical-scenario-generation/test/sequence_04/04_image2_clip.avi'
    videoCap = cv2.VideoCapture(videoFilename)
    import pickle
    raw = pickle.load(open('../../debug_lane_edit.pkl','rb'))
    lane2d = raw[0]
    param = raw[1]
    #app = LaneEditFrame(root, videoCap, lane2d, param, width=1200, height=600)
    app = LaneEditFrame(root, videoCap, dict(), param, width=1200, height=600)
    root.mainloop()