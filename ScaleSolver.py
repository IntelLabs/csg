import os, sys
from config import ROS_PATH
if ROS_PATH in sys.path:
	sys.path.remove(ROS_PATH)

from tkinter import *
from tkinter import filedialog, ttk, scrolledtext, Menu, messagebox
from PIL import ImageTk, ImageOps
from PIL import Image as PILImage 
import pickle
import numpy as np
import cv2
import math
from src.ui.drag_rect import DragRect
from src.ui.color import marker_colors,get_hex_by_index,get_finegrained_color
from src.ui.dialog import View_AskDistance, View_AskIDLabel
from src.ui.video_clipper import VideoClipperWindow
from src.ui.lane_frame import LaneSettingFrame
from src.ui.canvas import ZoomFrame  # , ZoomFrame2
from src.ui.image_utils import *
from src.ui.drag_line import DragLine
#from src.calibration.calib_main import camera_clibration_canvas
#from src.detect.detect_factory import Detecter
#from src.track.track_factory import  TRACKER
#from src.track.trajectory import postprocessing_trajectory,select_trajectory, auto_refine
#from src.road.road_main import initialize_bounary_points, generate_road
from src.error_check import multi_frame_error_check
from config import DET_CONFIG,TRACK_CONFIG, SOURCE
from src.scale.PoseData import LoadPoseData
from src.scale.CameraCal import CamCal
from src.scale.utils import *
from src.file_parser import *
import subprocess
import argparse
'''
Note: 
All coordinates loaded in variables are canvas coordiantes(for drawing)
Saved coordinates are pixel-based, so need conversion when save/load

'''

def parse_args():
    parser = argparse.ArgumentParser(
        description='Set the video name')

    parser.add_argument('--videoname', type=str,
                        help='path to the video', required=True)

    return parser.parse_args()

CANVAS_WIDTH = 640
CANVAS_HEIGHT = 480
ORIENTATION_LIST =  ['North', 'West', 'South', 'East']
RADIUS = 7#to draw oval in road
LINEWIDTH = 2 #draw line
SMALL_CANVAS_SIZE=200

SHOW_ZOOMED_FOCUS = True

class MainGUI:
	def __init__(self, master,videoname):
		self.parent = master
		self.parent.title("ScaleSolver v0.1")

		self.canvasFrame = Frame(self.parent)
		self.tabFrame = Frame(self.parent)
		self.statusFrame = Frame(self.parent)
		self.titleFrame = Frame(self.parent)
		self.titleFrame.grid(row=0, column=0, columnspan=2, sticky="ew")
		self.canvasFrame.grid(row=1, column=0, sticky="n")
		self.tabFrame.grid(row=1, column=1, sticky="nsew")
		self.statusFrame.grid(row=3, column=0, columnspan=2, sticky="ew")

		self.cameraView = StringVar()
		self.detectionModel = StringVar()
		self.enableTrajDisp = IntVar()

		# ----------------- GUI stuff ---------------------
		self.titleLabel = Label(self.titleFrame, text="Video:",  anchor=W)
		self.openBtn = Button(self.titleFrame, text="Open Video",command=self.open_video)
		self.SLAMBtn = Button(self.titleFrame, text="Mono SLAM", command=self.start_SLAM)
		self.titleLabel.pack(fill=X, side=LEFT, expand=1)
		#self.cameraViewBox.pack(fill=X, side=RIGHT, padx=5, expand=0)
		self.SLAMBtn.pack(fill=X, side=RIGHT, padx=5, expand=0)
		self.openBtn.pack(fill=X, side=RIGHT, padx=5, expand=0)
		self.scale_solver = None




		self.canvasDisplayFrame = ttk.Frame(self.canvasFrame)
		self.canvasDisplayFrame.grid(row=0, column=0, columnspan=2)
		self.videoNavigationFrame = ttk.Frame(self.canvasFrame, width=CANVAS_WIDTH-SMALL_CANVAS_SIZE-50)
		self.videoNavigationFrame.grid(row=1, column=0,padx=5, sticky="w")
		self.resultDisplayFrame = ttk.Frame(self.canvasFrame)
		self.resultDisplayFrame.grid(row=1, column=1, padx=10, pady=10, sticky="e")

		self.canvasPaintFrame = ZoomFrame(self.canvasDisplayFrame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT,
										 background="#90AFC5")  # cursor='tcross'
		self.frameLabel = Label(self.videoNavigationFrame, text="Frame ")
		self.gotoFirstBtn = Button(self.videoNavigationFrame, text="|<", command=self.goto_first)
		self.gotoPrev10Btn = Button(self.videoNavigationFrame, text="<<", command=self.goto_prev10)
		self.gotoPrevBtn = Button(self.videoNavigationFrame, text="<", command=self.goto_prev)
		self.playBtn = Button(self.videoNavigationFrame,text="|>", command=self.play)
		self.gotoNextBtn = Button(self.videoNavigationFrame, text=">", command=self.goto_next)
		self.gotoNext10Btn = Button(self.videoNavigationFrame, text=">>", command=self.goto_next10)
		self.gotoLastBtn = Button(self.videoNavigationFrame, text=">|", command=self.goto_last)
		self.clipperIcon = PhotoImage(file="./icon/clipper.jpg")
		self.videoEditBtn = Button(self.videoNavigationFrame, image=self.clipperIcon, command=self.video_edit)
		
		self.frameLabel.pack(fill=BOTH, side=TOP)
		self.frameLabel.config(font=("Helvetica", 16))#Helvetica, Rouge
		self.gotoFirstBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.gotoPrev10Btn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.gotoPrevBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.playBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.gotoNextBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.gotoNext10Btn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.gotoLastBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
		self.videoEditBtn.pack(fill=X, side=LEFT, padx=5, expand=0)

		self.animationDisplayCanvas = Canvas(self.resultDisplayFrame, background='white',
											width=SMALL_CANVAS_SIZE,height=SMALL_CANVAS_SIZE)
		self.animationDisplayCanvas.pack(fill=X, side=RIGHT)

	   # s =ttk.Style()
	   # s.configure('', tabposition='en')
		# self.tabFrame.pack_propagate(False)
		self.tabControl = ttk.Notebook(self.tabFrame)
		self.tabCalib = ttk.Frame(self.tabControl)
		self.tabControl.add(self.tabCalib, text='ScaleSolve')
		self.tabControl.pack(expand=1, fill="both")  # Pack to make visible

		# tab-calibration
	   # self.calHlsBtn = Button(self.tabCalib, text='HLS_Image', command=self.show_hls)
	   # self.calHlsBtn.pack(fill=X, side=TOP, pady=5)
		self.calPointEditFrame = Frame(self.tabCalib)
		self.calPointEditFrame.pack(fill=X,pady=5, side=TOP)
		#Button(self.tabCalib, text="MonoSLAM", command=self.clean_cal_selections).pack(fill=X, side=TOP)  # , pady=1)
		self.calEditPLRPBtn = Button(self.calPointEditFrame, text="Adjust",command=self.select_edit_add,
								  disabledforeground = 'red')
		self.calEditPLRPBtn.grid(row=0, column=1, rowspan=2, sticky='ew')
		self.calAddRPBtn = Button(self.calPointEditFrame, text="Insert Ref Points",command=self.select_ref_points,
								  disabledforeground="red")
		self.calAddRPBtn.grid(row=1, column=0, sticky='we')
		self.calPointEditFrame.columnconfigure(0, weight=1)
		self.calPointEditFrame.columnconfigure(1, weight=1)

		Button(self.tabCalib, text="Re-Selection",command=self.clean_cal_selections).pack(fill=X, side=TOP)  # , pady=1)
		Button(self.tabCalib, text="AutoCalibrate", command=self.cam_calib).pack(fill=X, side=TOP)  # , pady=1)
		Button(self.tabCalib, text="Save Results",command=self.calib_save_results).pack(fill=X, side=TOP)  # , pady=1)
		#Button(self.tabCalib, text="Display Results",command=self.calib_disp_results).pack(fill=X, side=TOP)  # , pady=1)
		ttk.Separator(self.tabCalib, orient=HORIZONTAL).pack(fill=X, pady=10, side=TOP)  # grid(..., sticky="ew")
		self.calFrameLbl = Label(self.tabCalib, text="")
		self.calFrameLbl.pack(fill=X, side=TOP)

		Label(self.tabCalib, text="Selected Reference Lines").pack(fill=X, side=TOP)
		self.calRPListBox = Listbox(self.tabCalib, width=40, height=5)
		self.calRPListBox.pack(fill=X, side=TOP)
		Label(self.tabCalib, text="Scaled Map",font=("Helvetica", 12),background='green', fg='white').pack(fill=X, side=TOP)

		from matplotlib.backends.backend_tkagg import (
			FigureCanvasTkAgg, NavigationToolbar2Tk)
		# Implement the default Matplotlib key bindings.
		from matplotlib.backend_bases import key_press_handler
		from matplotlib.figure import Figure

		self.Mapfig = Figure(figsize=(5, 4), dpi=100)
		#fig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t), 'o')
		# canvas.draw()
		self.ax_map = self.Mapfig.add_subplot(111)
		self.ax_map.set_xlim([-50,50])
		self.ax_map.set_ylim([-50,500])

		self.MapCanvas = FigureCanvasTkAgg(self.Mapfig, master=self.tabCalib)  # A tk.DrawingArea.

		self.MapCanvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

		#self.calRstListBox = Listbox(self.tabCalib, height=20)
		#self.calRstListBox.pack(fill=X, side=TOP)
		# status frame
		self.statusLabel = Label(self.statusFrame, text="Status:", bd=1, relief=SUNKEN, anchor=W)
		self.statusLabel.pack(fill=X, side=LEFT, expand=True)
		self.statusFrameLabel = Label(self.statusLabel, text="Frame", bd=1, relief=SUNKEN, anchor=E)
		self.statusFrameLabel.pack(fill=BOTH, side=RIGHT, expand=False)

		self.parent.grid_rowconfigure(1, weight=1)
		self.parent.grid_columnconfigure(1, weight=1)
		self.bind_events()


		# ----------------- Variables---------------------
		self.videoFilename = videoname
		self.videoCap = None
		self.totalFrameNum = 0
		self.currFrameId = None
		self.calFrameID = None  # frame id to do clibration
		self.roadFrameID = None  # frame id to do road reconstruction
		self.imgRaw = None  # cv mode(bgr)
		self.imgDisp = None
		self.imgScale = None
		self.imgPadding = None
		self.imgHLS = None
		self.imgHLSDisp = None
		self.imgRoad = None
		self.imgRoadDisp = None
		self.tkimg = None
		self.detector = None
		self.tracker = None
		self.ObjectList = None
		self.ObjectList_pixel = None
		self.trajectory = None
		self.nextObjectID = 1

		self.calParaLines = []  # canvas_coords,group of 4 points
		self.calRefPoints = []  # canvas_coords,group of 2 points
		self.calParaLinesObjIds = []
		self.calRefPointsObjIds = []
		self.cameraParam = None
		self.roadBoundPts = dict()  # save selected roudn poits
		self.roadLaneNum ={'ne':1, 'nw':1, 'wn':1, 'ws':1, 'sw':1, 'se':1,'es':1,'en':1}#default
		self.dragRectObjs = None
		self.labelList =['Car', 'Bus','Pedestrian','Rider', 
						'Truck', 'Motorcycle','Bicycle','Tricycle']#TODO: read/write to file 
		
		# mode-related variables
		self.currHitTab = ""  # text
		self.hlsImageMode = False
		self.roadImageMode = False #road or hls image
		self.playMode = True #play or pause
		self.calAddPLMode = True #para/ref to be added
		self.calAddEditMode = True #edit or insert ref/para
		self.InsertObjectMode = False #add or edit objects in canvas
		

		# temporay variables used to recording intermediate status
		self.tempParaLineObjIds = []
		self.tempDragData = None  # drag
		self.tempInsertObjectData = dict()
		self.tempParallelData = dict()
		self.tempRefPointData = dict() #temp when drawing line(parallel mode)
		self.tempPLLine = [] #save temporary line(first) when drawing 2 parallel lines
		
		###new added
		self.tempRoadData = None  # road
		self.SamplesFlag_N = False
		self.SamplesFlag_W = False
		self.SamplesFlag_S = False
		self.SamplesFlag_E = False
		self.RoadDataList_N=[]
		self.RoadDataList_W=[]
		self.RoadDataList_S=[]
		self.RoadDataList_E=[]
		
		
		#icon image
		self.icon_image_size=(20,20)
		warning_icon = Image.open('icon/warning.png')
		warning_thumb=ImageOps.fit(warning_icon, self.icon_image_size, Image.LANCZOS)
		self.warning_tkimg = ImageTk.PhotoImage(warning_thumb)
		pass_icon = Image.open('icon/pass.png')
		pass_thumb=ImageOps.fit(pass_icon, self.icon_image_size, Image.LANCZOS)
		self.pass_tkimg = ImageTk.PhotoImage(pass_thumb)
		empty_arr = np.zeros([self.icon_image_size[1], self.icon_image_size[0], 3],dtype=np.uint8)
		empty_arr.fill(255)
		self.empty_tkimg = ImageTk.PhotoImage(Image.fromarray(empty_arr))
		self.open_video(self.videoFilename)

	def bind_events(self):  # bind all messages
		self.parent.bind("<Key-Left>", self.goto_prev)
		self.parent.bind("<Key-Right>", self.goto_next)
		self.parent.bind("Escape", self.cancel_bbox)

		self.tabControl.bind("<<NotebookTabChanged>>", self.tab_click)

		self.canvasPaintFrame.canvas.bind("<Button-1>", self.mouse_click)
		self.canvasPaintFrame.canvas.bind("<Motion>", self.mouse_move)#, "+")
		self.canvasPaintFrame.canvas.bind("<B1-Motion>", self.mouse_drag)
		self.canvasPaintFrame.canvas.bind("<ButtonRelease-1>", self.mouse_release)
		

		#self.calPLListBox.bind('<Key>', self.edit_parallel_line_keypress)
		self.calRPListBox.bind('<Double-Button>', self.edit_reference_points)
		self.calRPListBox.bind('<Key>', self.edit_reference_points_keypress)
		'''
		self.detObjectListBox.bind('<Double-Button>', self.edit_object_label_id)
		self.detObjectListBox.bind('<<ListboxSelect>>', self.listbox_select)
		self.detObjectListBox.bind('<Key>', self.object_listbox_keypress)
		
		self.frameTreeView.bind('<Double-Button>', self.treeview_click)
		'''
	def open_video(self, filename=None):  
	   # self.parent.focus()
		if filename is None: 
			self.videoFilename = filedialog.askopenfilename(title="Select Video", filetypes=(("video files", "*.mp4"), ("video files", "*.avi"),
																						 ("all files", "*.*")))
			if not self.videoFilename: 
				return 
		else: 
			self.videoFilename = filename
		if not isinstance(self.videoFilename, str):
			return
		if not os.path.exists(self.videoFilename):
			messagebox.showerror("Error", "Cannot find file")
			self.videoFilename = None
			return
		try: 
			self.videoCap.release()
		except: #do nothing
			pass    
		self.videoCap = cv2.VideoCapture(self.videoFilename)
		if not self.videoCap.isOpened():
			messagebox.showerror("Error", "Cannot open video file")
			self.videoFilename = None
			self.videoCap = None
			return
		#self.clean_draw_objects()
		#kf_saver = os.path.join(self.videoFilename + "_KeyFrameTrajectory.txt")
		#times_saver = os.path.join(self.videoFilename + "_times.txt")
		#gt_saver = os.path.join(self.videoFilename + "_gt.txt")
		#if not os.path.exists(kf_saver):
			#return

		kf_saver = os.path.join(os.path.dirname(self.videoFilename),
									 'KeyFrameTrajectory.txt')
		times_saver = os.path.join(os.path.dirname(self.videoFilename),
									  'times.txt')

		gt_saver = os.path.join(self.videoFilename + "_gt.txt")

		if os.path.exists(kf_saver) and os.path.exists(times_saver):
			self.kf_idx_list, self.kf_pose_data = LoadPoseData(kf_saver,times_saver)
			self.canvasPaintFrame.canvas.delete('all')
			self.reset_modes(True)
			self.totalFrameNum = int(self.videoCap.get(7))
			self.currFrameId = 0
			self.titleLabel.config(text="Video Path: "+self.videoFilename)
			self.load_image_data(self.kf_idx_list[self.currFrameId])
			if os.path.exists(gt_saver):
				self.load_pose_gt(gt_saver)
				self.ax_map.plot(self.GT_Position[:, 0], self.GT_Position[:, 1], 'o')
				self.ax_map.legend(['GT_map'])
				self.MapCanvas.draw()
			self.draw_image(self.imgDisp)
			self.draw_annotations()
			#self.generate_thumbnail_view()
			self.update_message_boxes()
			self.statusFrameLabel.config(text ='Frame {}/{}'.format(self.kf_idx_list[self.currFrameId], self.totalFrameNum))
			self.frameLabel.config(text ='Frame {}'.format(self.kf_idx_list[self.currFrameId]))
		else:
			messagebox.showinfo(
				title='Warning', message='Please do SLAM first')
			return


	def load_pose_gt(self,gt_file):
		self.GT_Pose = read_kitti_poses_file(gt_file)
		self.GT_Position = np.asarray([[pose[0,3],pose[2,3]] for pose in self.GT_Pose])
	def start_SLAM(self):

		cmd = "cd ./src/SLAM/ && ./mono_kitti  ORBvoc.txt KITTI00-02.yaml  test2"
		#"./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER"
		subprocess.Popen(cmd, shell=True, executable="/bin/bash")
		print("gen enhance file cmd", cmd)
		print("generate test cases for scenario %s" % casename + "_00")

	def load_image_data(self, frame_id):
		print(frame_id)
		self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
		_, self.imgRaw = self.videoCap.read()
		self.imgDisp, _, self.imgScale, self.imgPadding = fit_image_to_canvas(self.imgRaw,
										desired_width=CANVAS_WIDTH, desired_height=CANVAS_HEIGHT)

	def draw_image(self, image_data):
		if image_data is None:
			return
		rgb = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
		self.tkImg = ImageTk.PhotoImage(PILImage.fromarray(rgb))
		self.canvasPaintFrame.load_image(PILImage.fromarray(rgb))


	def draw_annotations(self):
		# redraw all annotaitons(detection, calibration, road)
		if self.imgRaw is None: 
			return 
		if self.currHitTab=='Detect&Track':
			if self.ObjectList is None: 
				self.ObjectList = [[] for _ in range(self.totalFrameNum)]
				self.ObjectList_pixel = [[] for _ in range(self.totalFrameNum)]
				return
			curr_anno = self.ObjectList[self.currFrameId]
			self.dragRectObjs = []
			for anno in curr_anno: 
				id =anno[0]
				if 1: 
					color = get_hex_by_index(id)
				else: #debug mode, color differs on source
					src = anno[-1]
					if src ==0: #auto-det
						color = "#FF0000"#red
					elif src==1: #track
						color = '#FFFF00'#yellow
					elif src==2: #manual 
						color = '#03ecfc'#blue
					else: #others
						color = '#008000'#green
				label_str = self.labelList[anno[1]]
				textstr = "{}  {}".format(label_str, id)
				points =[[anno[2], anno[3]],[anno[4], anno[5]]]
				ls = DragRect(self, self.canvasPaintFrame.canvas,points, color, 
							 title = textstr, objectID=id)
				self.dragRectObjs.append(ls)
			self.show_trajectory(False)
			self.show_trajectory(True)
		elif self.currHitTab=='ScaleSolve':
			if not self.calFrameID is None: 
				self.calFrameLbl.configure(text='Frame {}'.format(self.calFrameID+1))
			if self.currFrameId == self.calFrameID:
				self.calParaLinesObjIds = []
				for item in self.calParaLines:
					temp_objs = []
					oid = item[0][0]
					temp_objs = []
					for i in range(2):
						title = '{}({})'.format(oid, i+1)
						points = item[i][1:]
						para_line = DragLine(self, self.canvasPaintFrame.canvas, points, 
									 oid, color='red',tag='parallel_line',
									 title=title)
						temp_objs.append(para_line)
					self.calParaLinesObjIds.append(temp_objs)   
				self.calRefPointsObjIds=[]
				for item in self.calRefPoints:
					temp_objs = []
					oid = item[0]
					points = item[1:5]
					title = '{}'.format(oid)
					ref_line = DragLine(self, self.canvasPaintFrame.canvas, 
										points, oid, color='yellow', 
										tag ='reference_line', title=title)
					self.calRefPointsObjIds.append(ref_line)

		elif self.currHitTab=='Road Reconstruction':
			if self.roadFrameID == self.currFrameId:
				if bool(self.roadBoundPts): 
					self.bound_oval_id=dict()
					for key in self.roadBoundPts:
						x,y =self.roadBoundPts[key][0], self.roadBoundPts[key][1]
						self.bound_oval_id[key] = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
																 x + RADIUS, y + RADIUS,
																fill=marker_colors[key], tags='boundary_points')
					self.canvasPaintFrame.canvas.tag_bind("boundary_points", "<ButtonPress-1>", self.drag_oval_start)
					self.canvasPaintFrame.canvas.tag_bind("boundary_points", "<ButtonRelease-1>", lambda event, tag='boundary_points':self.drag_oval_stop(event, tag))
					self.canvasPaintFrame.canvas.tag_bind("boundary_points", "<B1-Motion>", lambda event, tag='boundary_points':self.drag_oval(event, tag))
				if bool(self.roadLaneNum): 
					self.laneSettingFrame.set_lane_num(self.roadLaneNum)
		
	def tab_click(self, event):
		selection = event.widget.select()
		tab = event.widget.tab(selection, "text")
		if 'Detect' in tab:
			self.canvasPaintFrame.set_zoomable(True)
		else:
			self.canvasPaintFrame.set_zoomable(False)
		self.currHitTab = tab
		self.clean_draw_objects()
		self.reset_modes(False)
		self.draw_image(self.imgDisp)
		self.draw_annotations()
		self.update_message_boxes()

	def update_dragrect(self, target_object_id): 
		#update annotation once dragrect is changed 
		for i, obj in enumerate(self.dragRectObjs): 
			if obj.objectID ==target_object_id: #find object
				drag_rect_points = obj.points
				canvas_points = self.canvasPaintFrame.get_pixel_values(drag_rect_points)
				pixel_points = convert_canvas_to_pixel(canvas_points, self.imgScale, self.imgPadding)
				self.ObjectList[self.currFrameId][i][2:6]= np.asarray(canvas_points).ravel().astype(int)
				self.ObjectList_pixel[self.currFrameId][i][2:6]=np.asarray(pixel_points).ravel().astype(int)
				self.ObjectList[self.currFrameId][i][-1] = SOURCE['Manual']
				self.ObjectList_pixel[self.currFrameId][i][-1] = SOURCE['Manual']
				break
		self.update_message_boxes()#update framelist box info. 
	def update_line(self, target_object_id, type):
		if type=='parallel_line': 
			for i, lines in enumerate(self.calParaLines): 
				if lines[0][0]==target_object_id: 
					for j in range(2): 
						points = self.calParaLinesObjIds[i][j].points
						self.calParaLines[i][j][1:] = np.asarray(points).ravel()
					break 
		elif type=='reference_line': 
			for i, line in enumerate(self.calRefPoints): 
				if line[0]==target_object_id: 
					points = self.calRefPointsObjIds[i].points
					self.calRefPoints[i][1:5] = np.asarray(points).ravel()
					break 
		self.update_message_boxes()
	def update_message_boxes(self):
		if self.currHitTab =='Detect&Track':
			if self.ObjectList is None: 
				return 
			#update object listbox 
			self.detObjectListBox.delete(0,END)
			curr_objs = self.ObjectList[self.currFrameId]
			for obj in curr_objs: 
				temp_str = '{} : {}'.format(obj[0], self.labelList[obj[1]])
				self.detObjectListBox.insert(END, temp_str)
			valid_arr = multi_frame_error_check(self.ObjectList)
			tree = self.frameTreeView.get_children()
			if len(tree)>0: 
				for i in range(self.totalFrameNum): 
					src_list = [item[-1] for item in self.ObjectList[i]]
					src_list = list(set(src_list))
					info_str = ''
					if SOURCE['Auto'] in src_list: 
						info_str += ' A '
					else: 
						info_str += '   '
					if SOURCE['Track'] in src_list: 
						info_str += ' T '
					else: 
						info_str += '   '
					if SOURCE['Manual'] in src_list: 
						info_str += ' M '
					else: 
						info_str += '   '
					v = valid_arr[i]
					if v ==0: 
						self.frameTreeView.item(tree[i],values=('Frame {}'.format(i+1),info_str),
												image=self.warning_tkimg)
					elif v==1: 
					   self.frameTreeView.item(tree[i],values=('Frame {}'.format(i+1),info_str),
												image=self.pass_tkimg)  
					else:
						self.frameTreeView.item(tree[i],values=('Frame {}'.format(i+1),info_str),
												image=self.empty_tkimg)  
		elif self.currHitTab=='ScaleSolve':
			# fill in calParaLines->calPLListBox
			#self.calPLListBox.delete(0, END)
			# each paraline has 4 points
			for lines in self.calParaLines:
				temp_str = '{}:'.format(lines[0][0])
				for j in range(1):
					temp_str += '({},{})-({},{})-({})'.format(lines[j][1],lines[j][2],
												 lines[j][3],lines[j][4],lines[j][5])
					if j==0: 
						temp_str +='\t'
				self.calPLListBox.insert(END, temp_str)
			# fill in calRefPoints ->calRPListBox
			self.calRPListBox.delete(0, END)
			for i, line in enumerate(self.calRefPoints):
				temp_str = '{}: ({},{})-({},{})\t Distance={} \t KeyFrameNum={}'.format(
					line[0], line[1], line[2], line[3], line[4], line[5],line[6])
				self.calRPListBox.insert(END, temp_str)
			# todo: fill result pose
				
		elif self.currHitTab=='Road Reconstruction':
			pass #no message box in this tab

	def clean_draw_objects(self):
		# TODO: clean all annotations, keep image
		self.canvasPaintFrame.canvas.delete('dragRect')
		self.canvasPaintFrame.canvas.delete('parallel_line')
		self.canvasPaintFrame.canvas.delete('reference_line')
	   # self.canvasPaintFrame.canvas.delete('line')
		self.canvasPaintFrame.canvas.delete('boundary_points')
		self.show_trajectory(False)

	def reset_modes(self, clean_results=False):
		#self.calHlsBtn["text"] = "HLS Image"
		self.hlsImageMode = False
		self.calAddPLMode = True #EDIT_MODE
		self.calAddEditMode = True #edit reference point
		self.calEditPLRPBtn['state'] = 'disable'
		self.calAddRPBtn['state'] = 'normal'
		self.pauseMode = False
		self.GT_Pose = None
		self.GT_Position =None

		#default button color 
		self.calEditPLRPBtn.configure(bg='#6495ed')
		#self.modifyObjectBtn.configure(bg='#6495ed')
		c = self.parent.cget('bg')
		self.calAddRPBtn.configure(bg=c)
		#self.calAddPLBtn.configure(bg=c)
		#self.insertObjectBtn.configure(bg=c)

		if clean_results: 
			#self.ObjectList = None
			#self.ObjectList_pixel = None
			self.cameraParam = None
			#self.roadBoundPts = dict()
			#self.roadLaneNum = {'ne':1, 'nw':1, 'wn':1, 'ws':1, 'sw':1, 'se':1,'es':1,'en':1}#default
			self.calParaLines = []  # canvas_coords,group of 4 points
			self.calRefPoints = []  # canvas_coords,group of 2 points
			self.calParaLinesObjIds = []
			self.calRefPointsObjIds = []
			#self.trajectory = None
			


	# --------tab1:-Calibration-----------
	def show_hls(self):
		if self.imgRaw is None:
			return
		self.clean_draw_objects()
		self.hlsImageMode = not self.hlsImageMode
		if self.hlsImageMode:
		   # self.calHlsBtn["text"] = "Original Image"
			self.imgHLS = convert_hls_image(self.imgRaw)
			# note: dim(self.imgHLS)== dim(self.imgRaw)
			self.imgHLSDisp = resize_padding_image(
				self.imgHLS, self.imgScale, self.imgPadding)
			self.draw_image(self.imgHLSDisp)
		else:
		  #  self.calHlsBtn["text"] = "HLS Image"
			self.draw_image(self.imgDisp)
			self.draw_annotations()
	
	def select_edit_add(self): 
		c = self.parent.cget('bg')
		self.calAddEditMode = not self.calAddEditMode
		if not self.calAddEditMode: 
			self.calEditPLRPBtn['state']='normal'
			#self.calAddPLBtn['state'] = 'disable'
			self.calAddRPBtn['state'] = 'disable'
		else: 
			self.calEditPLRPBtn['state']='disable'
			self.calAddRPBtn['state']='normal'
			#self.calAddPLBtn['state']='normal'
			self.calEditPLRPBtn.configure(bg='#6495ed')
			#self.calAddPLBtn.configure(bg=c)
			self.calAddRPBtn.configure(bg=c)

		self.statusLabel.config(text = "Mode: Edit Lines")
		for objs in self.calParaLinesObjIds:
			for obj in objs:  
				obj.freeze(False)
		for obj in self.calRefPointsObjIds: 
			obj.freeze(False)
		if len(self.tempPLLine)>0: 
			self.tempPLLine[0].erase()
			self.tempPLLine = []
	def select_parallel_line(self):
		if self.imgRaw is None: 
			return
		c = self.parent.cget('bg')
		self.calAddEditMode = False # not edit mode 
		self.calAddPLMode = True #insert parallel 
		self.calAddPLBtn['state'] = 'disable'
		self.calEditPLRPBtn['state'] = 'normal'
		self.calAddRPBtn['state']='normal'
		self.calAddPLBtn.configure(bg='#6495ed')
		self.calEditPLRPBtn.configure(bg=c)
		self.calAddRPBtn.configure(bg=c)
		self.calFrameID = self.currFrameId
		self.statusLabel.config(text = "Mode: insert paralle lines")
		for objs in self.calParaLinesObjIds:
			for obj in objs:  
				obj.freeze(True)
		for obj in self.calRefPointsObjIds: 
			obj.freeze(True)
			
	def select_ref_points(self):
		if self.imgRaw is None: 
			return
		c = self.parent.cget('bg')
		self.calAddEditMode = False # not edit mode 
		self.calAddPLMode = False #insert parallel 
		#self.calAddPLBtn['state'] = 'normal'
		self.calEditPLRPBtn['state'] = 'normal'
		self.calAddRPBtn['state']='disable'    
		self.calAddRPBtn.configure(bg='#6495ed')
		self.calEditPLRPBtn.configure(bg=c)
		#self.calAddPLBtn.configure(bg=c)

		self.calFrameID = self.currFrameId
		self.statusLabel.config(text = "Mode: insert reference points")
		for objs in self.calParaLinesObjIds:
			for obj in objs:  
				obj.freeze(True)
		for obj in self.calRefPointsObjIds: 
			obj.freeze(True)
		if len(self.tempPLLine)>0: #click when 1 line is draw, another is not drw
			self.tempPLLine[0].erase()
			self.tempPLLine = []
	def clean_cal_selections(self):  # re-select
		self.clean_draw_objects()
		self.calParaLines = []
		self.calRefPoints = []
		self.tempPLLine = []
		self.tempDragData = None  # drag boundary points
		self.update_message_boxes()
		return

	def cam_calib(self):

		if len(self.calRefPoints) < 1:
			messagebox.showinfo(
				"Info", "Please select at least 1 reference line")
			return

		SegNodes = []
		NodeDist = []
		Pose_list = []
		self.scale_solver = CamCal()
		width_scale = 1
		height_scale = 1
		for ref_line in self.calRefPoints:
			print(ref_line)
			for i in range(2):
				canvas_x, canvas_y = ref_line[1 + 2 * i], ref_line[1 + 2 * i + 1]
				pixel_x, pixel_y = get_pixel_coordinate(canvas_x, canvas_y, self.imgScale, self.imgPadding)
				SegNodes.append(np.array([pixel_x * width_scale, pixel_y * height_scale]))
			NodeDist.append([float(ref_line[5])])
			print(ref_line[-1])
			Pose_list.append(self.kf_pose_data[ref_line[-1]])


		#print(SegNodes)
		#print(NodeDist)
		#print(Pose_list)
		self.scale_solver.process(SegNodes, NodeDist,Pose_list)
		print(self.scale_solver.CamParam.Scale_X, self.scale_solver.CamParam.Scale_Z, self.scale_solver.CamParam.Ty)


		'''
		self.cameraParam = camera_clibration_canvas(self.calParaLines, self.calRefPoints,
													(self.imgRaw.shape[1],
													 self.imgRaw.shape[0]),
													self.imgScale, self.imgPadding)
		'''
		self.update_message_boxes()
		#####show the result map#####
		position_list=np.zeros((2,len(self.kf_pose_data)))
		for i in range(len(self.kf_pose_data)):
			position_list[0, i]=self.kf_pose_data[i][0,3]*self.scale_solver.CamParam.Scale_X
			position_list[1, i] = self.kf_pose_data[i][2, 3] * self.scale_solver.CamParam.Scale_Z
		#t = np.arange(0, 3, .01)
		#self.Mapfig.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t), 'o')
		self.ax_map.plot(position_list[0,:],position_list[1,:],'o')
		#self.ax_map.scatter(t, 2 * np.sin(2 * np.pi * t), color='g')
		#self.ax_map.scatter(t, 2 * np.cos(2 * np.pi * t), color='blue')
		self.ax_map.legend(['GT_map','scaled_map'])

		self.MapCanvas.draw()

	def calib_save_results(self):
		result_saver = os.path.join(os.path.dirname(self.videoFilename), 'ScaleSolver_result.txt')
		with open(result_saver, 'w+') as f:
			f.write('ScaleX: ' + str(self.scale_solver.CamParam.Scale_X[0]) + '\n')
			f.write('ScaleZ: ' + str(self.scale_solver.CamParam.Scale_Z[0]) + '\n')
			f.write('ScaleY: ' + str(self.scale_solver.CamParam.Ty[0]) + '\n')


	def load_calib_results(self):
		path = self.videoFilename+'_camcalib.pkl'
		if not os.path.exists(path):
			return
		raw = pickle.load(open(path, 'rb'))
		self.calFrameID = raw[0]
		paraline_pixels = raw[1]
		refpts_pixels = raw[2]
		self.cameraParam = raw[3]
		self.calParaLines = paraline_pixels
		for i, item in enumerate(paraline_pixels): 
			for j, line in enumerate(item): 
				pixel_coords=[[line[1], line[2],line[3], line[4]]]
				canvas_coords = convert_pixel_to_canvas(pixel_coords, self.imgScale, self.imgPadding)[0]
				self.calParaLines[i][j][1] = canvas_coords[0]
				self.calParaLines[i][j][2] = canvas_coords[1]
				self.calParaLines[i][j][3] = canvas_coords[2]
				self.calParaLines[i][j][4] = canvas_coords[3]
		self.calRefPoints = refpts_pixels
		for i, item in  enumerate(refpts_pixels): 
			pixel_coords=[[item[1], item[2], item[3], item[4]]]
			canvas_coords = convert_pixel_to_canvas(pixel_coords, self.imgScale, self.imgPadding)[0]
			self.calRefPoints[i][1] = canvas_coords[0]
			self.calRefPoints[i][2] = canvas_coords[1]
			self.calRefPoints[i][3] = canvas_coords[2]
			self.calRefPoints[i][4] = canvas_coords[3]
			

	def calib_disp_results(self, canvas_coord=None):
		if self.imgRaw is None: 
			return
		self.animationDisplayCanvas.delete('all')
		if SHOW_ZOOMED_FOCUS: #show cropped enlarged image, center on mouse
			try: 
				pixel_coord = convert_canvas_to_pixel([canvas_coord], self.imgScale, self.imgPadding)[0]
				raw_img_arr =  copy.deepcopy(self.imgRaw)
				raw_img_pil = Image.fromarray(cv2.cvtColor(raw_img_arr, cv2.COLOR_BGR2RGB))
				x,y = pixel_coord[0], pixel_coord[1]
				margin = 30
				zoom_img_crop = raw_img_pil.crop(((x - margin), (y - margin), (x + margin), (y + margin)))
				zoom_img_crop = zoom_img_crop.resize((SMALL_CANVAS_SIZE, SMALL_CANVAS_SIZE))
				self.imgZoomTk = ImageTk.PhotoImage(zoom_img_crop)
				self.animationDisplayCanvas.create_image(0, 0, image=self.imgZoomTk, anchor=NW)
				#hline 
				self.animationDisplayCanvas.create_line(0, SMALL_CANVAS_SIZE/2, 
														SMALL_CANVAS_SIZE, SMALL_CANVAS_SIZE/2, 
														width=2)
				#vline
				self.animationDisplayCanvas.create_line(SMALL_CANVAS_SIZE/2, 0,
														SMALL_CANVAS_SIZE/2, SMALL_CANVAS_SIZE, 
														width=2)
			except: 
				pass
		else: 
			if self.cameraParam is None:
				return 
			fpitch = self.calPitchScale.get()
			froll = self.calRollScale.get()
			fyaw = self.calYawScale.get()
			#note the perspective image is a square image, different resolution with raw image
			self.imgPerspectiveTk, self.cameraParm = convert_perspective_image(self.imgRaw, 
										froll, fpitch, fyaw, self.cameraParam,
										canvas_size=SMALL_CANVAS_SIZE)
			self.animationDisplayCanvas.create_image(0,0,image=self.imgPerspectiveTk, 
													 anchor='nw')
			
	def show_trajectory(self, show=True):
		if not show: 
			self.canvasPaintFrame.canvas.delete('trajectory')
			return

		bshow_traj = self.enableTrajDisp.get()
		if (self.ObjectList is None) or (self.trajectory is None): 
			return
		if bshow_traj==1:
			curr_objs = self.ObjectList[self.currFrameId]
			if len(curr_objs)<=0: 
				return 
			for i, obj in enumerate(curr_objs): 
				pts = select_trajectory(self.trajectory, obj[0], self.currFrameId)
				alpha_color = self.dragRectObjs[i].fill_color
				x_prev, y_prev = 0, 0
				for j, pt in enumerate(pts):
					x,y = pt[0],pt[1]
					self.canvasPaintFrame.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, 
											fill=alpha_color, tag='trajectory')
					if j >=1: 
						self.canvasPaintFrame.canvas.create_line(x_prev, y_prev, x, y, 
										fill=alpha_color, width=1, tag='trajectory')
					x_prev, y_prev = x,y
		else: 
			self.canvasPaintFrame.canvas.delete('trajectory')


	# --------Mouse event capture -----------

	def edit_reference_points(self, event):
		#todo , calRefPoints
		w = View_AskDistance(self.parent)  # canvasPaintFrame)
		self.parent.wait_window(w.top)
		if w.value:
			widget = event.widget
			selection = widget.curselection()
			#value = widget.get(selection[0])
			self.calRefPoints[selection[0]][-1] = float(w.value)
			self.update_message_boxes()

	def edit_reference_points_keypress(self, event):
		#print( "You pressed"), print(repr(event.char))
		c = repr(event.char)
		# print(c)
		if c == "'\\x7f'":  # delete press
			result = messagebox.askquestion(
				"Delete", "Are You Sure?", icon='warning')
			if result == 'yes':
				widget = event.widget
				selection = widget.curselection()
				del self.calRefPoints[selection[0]]
				self.calRefPointsObjIds[selection[0]].erase()
				del self.calRefPointsObjIds[selection[0]]
				self.update_message_boxes()
		#print("ref points"), print(self.calRefPoints)

	def edit_parallel_line_keypress(self, event):
		c = repr(event.char)
		if c == "'\\x7f'":  # delete press
			result = messagebox.askquestion(
				"Delete", "Are You Sure?", icon='warning')
			if result == 'yes':
				widget = event.widget
				selection = widget.curselection()
				del self.calParaLines[selection[0]]
				for line in self.calParaLinesObjIds[selection[0]]: 
					line.erase()
				del self.calParaLinesObjIds[selection[0]]
				self.update_message_boxes()
	def mouse_move(self, event): 
		if not self.dragRectObjs is None: 
			for i in range(len(self.dragRectObjs)):  
				self.dragRectObjs[i].highlight(False)
		if SHOW_ZOOMED_FOCUS: 
			self.calib_disp_results(canvas_coord=[event.x, event.y])
	
	def mouse_click(self, event):
		if self.imgRaw is None: 
			return
		if self.currHitTab =='Detect&Track':# only work in detection tab
			if self.InsertObjectMode: #only work in insert  mode
				x = self.canvasPaintFrame.canvas.canvasx(event.x)
				y = self.canvasPaintFrame.canvas.canvasy(event.y)
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
														 x + RADIUS, y + RADIUS,
														 fill='red', tags='temp_oval')
				oval_id2 = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
														 x + RADIUS, y + RADIUS,
														 fill='red', tags='temp_oval')
				rect_id = self.canvasPaintFrame.canvas.create_rectangle(x,y,x,y, width=2,
														fill='', outline='red',
														tags='temp_rect')
				self.tempInsertObjectData = dict()
				self.tempInsertObjectData["startpoint"] = oval_id1
				self.tempInsertObjectData["startx"] = x
				self.tempInsertObjectData["starty"] = y
				self.tempInsertObjectData["x"] = x
				self.tempInsertObjectData["y"] = y
				self.tempInsertObjectData["endpoint"] = oval_id2
				self.tempInsertObjectData["rect"] = rect_id
			else: 
				self.tempInsertObjectData = dict() #clean
		elif self.currHitTab =='ScaleSolve': 
			if self.calAddPLMode and not self.calAddEditMode: 
				x = self.canvasPaintFrame.canvas.canvasx(event.x)
				y = self.canvasPaintFrame.canvas.canvasy(event.y)
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
														 x + RADIUS, y + RADIUS,
														 fill='red', tags='temp_oval')
				oval_id2 = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
														 x + RADIUS, y + RADIUS,
														 fill='red', tags='temp_oval')
				line_id = self.canvasPaintFrame.canvas.create_line(x,y,x,y, width=2,
														fill='red',
														tags='temp_line')
				self.tempParallelData = dict()
				self.tempParallelData["startpoint"] = oval_id1
				self.tempParallelData["startx"] = x
				self.tempParallelData["starty"] = y
				self.tempParallelData["x"] = x
				self.tempParallelData["y"] = y
				self.tempParallelData["endpoint"] = oval_id2
				self.tempParallelData["line"] = line_id
			elif not self.calAddEditMode and not self.calAddEditMode: 
				x = self.canvasPaintFrame.canvas.canvasx(event.x)
				y = self.canvasPaintFrame.canvas.canvasy(event.y)
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
														 x + RADIUS, y + RADIUS,
														 fill='yellow', tags='temp_oval')
				oval_id2 = self.canvasPaintFrame.canvas.create_oval(x - RADIUS, y - RADIUS,
														 x + RADIUS, y + RADIUS,
														 fill='yellow', tags='temp_oval')
				line_id = self.canvasPaintFrame.canvas.create_line(x,y,x,y, width=2,
														fill='yellow',
														tags='temp_line')
				self.tempRefPointData = dict()
				self.tempRefPointData["startpoint"] = oval_id1
				self.tempRefPointData["startx"] = x
				self.tempRefPointData["starty"] = y
				self.tempRefPointData["x"] = x
				self.tempRefPointData["y"] = y
				self.tempRefPointData["endpoint"] = oval_id2
				self.tempRefPointData["line"] = line_id
		elif self.currHitTab =='Road Reconstruction':# only work in detection tab
			x = self.canvasPaintFrame.canvas.canvasx(event.x)
			y = self.canvasPaintFrame.canvas.canvasy(event.y)
			
			if self.SamplesFlag_N:
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - 3, y - 3,
															 x + 3, y + 3,
															 fill='blue', tags='temp_oval')
				self.tempRoadData=[x,y]
			elif self.SamplesFlag_W:
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - 3, y - 3,
															 x + 3, y + 3,
															 fill='yellow', tags='temp_oval')
				self.tempRoadData=[x,y]
			elif self.SamplesFlag_S:
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - 3, y - 3,
															 x + 3, y + 3,
															 fill='green', tags='temp_oval')
				self.tempRoadData=[x,y]
			elif self.SamplesFlag_E:
				oval_id1 = self.canvasPaintFrame.canvas.create_oval(x - 3, y - 3,
															 x + 3, y + 3,
															 fill='red', tags='temp_oval')
				self.tempRoadData=[x,y]



	def mouse_drag(self, event):
		if self.currHitTab =='Detect&Track':# only work in detection tab
			if self.calAddPLMode and bool(self.tempInsertObjectData): #only work in insert  mode
				x = self.canvasPaintFrame.canvas.canvasx(event.x)
				y = self.canvasPaintFrame.canvas.canvasy(event.y)
				delta_x = event.x - self.tempInsertObjectData["x"]
				delta_y = event.y - self.tempInsertObjectData["y"]
				self.canvasPaintFrame.canvas.move(self.tempInsertObjectData["endpoint"], delta_x, delta_y)
				self.canvasPaintFrame.canvas.coords(self.tempInsertObjectData["rect"], 
												 self.tempInsertObjectData['startx'], self.tempInsertObjectData['starty'],
												 x,y)
				self.tempInsertObjectData["x"] = x
				self.tempInsertObjectData["y"] = y
			elif not self.InsertObjectMode: 
				self.tempInsertObjectData = dict()
		elif self.currHitTab=='ScaleSolve': 
			if self.calAddPLMode and not self.calAddEditMode and bool(self.tempParallelData): #only work in insert  mode
				x = self.canvasPaintFrame.canvas.canvasx(event.x)
				y = self.canvasPaintFrame.canvas.canvasy(event.y)
				delta_x = event.x - self.tempParallelData["x"]
				delta_y = event.y - self.tempParallelData["y"]
				self.canvasPaintFrame.canvas.move(self.tempParallelData["endpoint"], delta_x, delta_y)
				self.canvasPaintFrame.canvas.coords(self.tempParallelData["line"], 
												 self.tempParallelData['startx'], 
												 self.tempParallelData['starty'],
												 x,y)
				self.tempParallelData["x"] = x
				self.tempParallelData["y"] = y
			elif not self.calAddEditMode and not self.calAddEditMode and bool(self.tempRefPointData): 
				x = self.canvasPaintFrame.canvas.canvasx(event.x)
				y = self.canvasPaintFrame.canvas.canvasy(event.y)
				delta_x = event.x - self.tempRefPointData["x"]
				delta_y = event.y - self.tempRefPointData["y"]
				self.canvasPaintFrame.canvas.move(self.tempRefPointData["endpoint"], delta_x, delta_y)
				self.canvasPaintFrame.canvas.coords(self.tempRefPointData["line"], 
												 self.tempRefPointData['startx'], 
												 self.tempRefPointData['starty'],
												 x,y)
				self.tempRefPointData["x"] = x
				self.tempRefPointData["y"] = y

		if SHOW_ZOOMED_FOCUS: 
			self.calib_disp_results(canvas_coord=[event.x, event.y])

	def mouse_release(self, event):
		if self.imgRaw is None: 
			return
		need_update = False
		if self.currHitTab =='ScaleSolve':  # calibaration mode
			#curr_x, curr_y = event.x, event.y
			self.calFrameID = self.currFrameId
			self.calFrameLbl.configure(text='Frame {}'.format(self.calFrameID+1))
			if self.calAddPLMode and not self.calAddEditMode and bool(self.tempParallelData): 
				points = [self.tempParallelData['startx'], self.tempParallelData['starty'], 
						  self.tempParallelData['x'],self.tempParallelData['y']]
				if not math.sqrt((points[0]-points[2])*(points[0]-points[2])+
						 (points[1]-points[3])*(points[1]-points[3]))<10: #too small objects
					oid = 0
					for item in self.calParaLines: 
						oid = max(item[0][0], oid)
					oid = oid+1
					if len(self.tempPLLine)<1: 
						title = '{}(1)'.format(oid)
					elif len(self.tempPLLine)==1: 
						title='{}(2)'.format(oid)
					para_line = DragLine(self, self.canvasPaintFrame.canvas, points, 
										oid, color='red', 
										tag='parallel_line',
										title=title
										)
					para_line.freeze(True)
					self.tempPLLine.append(para_line)
					#if len(self.tempPLLine) ==2: #collect 2 lines
					temp_points = []
					for obj in self.tempPLLine:
						oid = obj.objectID
						pts = obj.points
						temp_points.append([oid, pts[0][0],pts[0][1],pts[1][0], pts[1][1]])
					self.calParaLines.append(temp_points)
					self.calParaLinesObjIds.append(self.tempPLLine)
					self.tempPLLine = []
					print('frame',self.currFrameId)
			elif not self.calAddPLMode and not self.calAddEditMode and bool(self.tempRefPointData): 
				print(self.imgScale, self.imgPadding)
				points = [self.tempRefPointData['startx'], self.tempRefPointData['starty'],
							 self.tempRefPointData['x'],self.tempRefPointData['y']]
				if not math.sqrt((points[0]-points[2])*(points[0]-points[2])+
							 (points[1]-points[3])*(points[1]-points[3]))<10: #too small objects
					w = View_AskDistance(self.parent)  # canvasPaintFrame)
					self.parent.wait_window(w.top)
					if w.value:
						oid = 0
						for item in self.calRefPoints: 
							oid = max(item[0], oid)
						oid = oid+1
						ref_line = DragLine(self, self.canvasPaintFrame.canvas, points, oid, color='yellow', 
											tag ='reference_line', 
											title=str(oid)
											)
						ref_line.freeze(True)
						self.calRefPointsObjIds.append(ref_line)
						self.calRefPoints.append([oid, points[0], points[1], points[2],points[3], w.value,self.currFrameId])
						p1,p2=get_pixel_coordinate(points[0], points[1], self.imgScale, self.imgPadding)
						p3,p4=get_pixel_coordinate(points[2], points[3], self.imgScale, self.imgPadding)
						#print(p1,p2,p3,p4)
			self.canvasPaintFrame.canvas.delete('temp_line')
			self.canvasPaintFrame.canvas.delete('temp_oval')
			self.tempRefPointData = dict()
			self.tempParallelData = dict()
			need_update = True     
			
		elif self.currHitTab =='Detect&Track':
			#create new dragRect
			if bool(self.tempInsertObjectData) and self.InsertObjectMode: 
				points = [[self.tempInsertObjectData["startx"],self.tempInsertObjectData["starty"]],
						[self.tempInsertObjectData["x"],self.tempInsertObjectData["y"]]]
				if abs(points[0][0]-points[1][0])<3 or abs(points[0][1]-points[1][1])<3: 
					self.cancel_bbox()
					self.tempInsertObjectData = dict()
					return #tiny object, might come from click not drag
				w = View_AskIDLabel(self.parent, self.labelList, 
									ObjectID=self.nextObjectID)  # canvasPaintFrame)
				self.parent.wait_window(w.top)
				if w.label is None: 
					self.cancel_bbox()
				else: 
					label = w.label 
					id = w.ObjectID
					textstr = "{}  {}".format(label, id)
					color = get_hex_by_index(id)
					ls = DragRect(self, self.canvasPaintFrame.canvas, points, 
								color, title=textstr, objectID = id)
					ls.freeze(True)
					self.labelList = w.labelList #update labelist
					label_id = self.labelList.index(label)
					points_true_canvas = self.canvasPaintFrame.get_pixel_values(points)
					new_obj = [id, label_id, points_true_canvas[0][0], points_true_canvas[0][1], 
								points_true_canvas[1][0], points_true_canvas[1][1], 1.0, SOURCE['Manual']]
					self.ObjectList, self.ObjectList_pixel, self.trajectory, max_obj_id= \
						 auto_refine(self.videoCap, self.parent, 
									 self.currFrameId, new_obj,
									 self.ObjectList, self.ObjectList_pixel, self.trajectory,
									 self.imgScale, self.imgPadding,'insert')
					if max_obj_id <=0: 
						self.nextObjectID = max(self.nextObjectID, id +1) #may not contiguous, if not reidenx when insert
					else: 
						self.nextObjectID = max(self.nextObjectID, max_obj_id+1)
					self.canvasPaintFrame.canvas.delete("temp_oval")
					self.canvasPaintFrame.canvas.delete("temp_rect")
					if self.dragRectObjs is None: 
						self.dragRectObjs = []
					self.dragRectObjs.append(ls)
					#may need to reset dragrect title, since 
					for i in range(len(self.ObjectList[self.currFrameId])):
						id, lbl_id=self.ObjectList[self.currFrameId][i][0], self.ObjectList[self.currFrameId][i][1]
						label_str = self.labelList[lbl_id]
						textstr = "{}  {}".format(label_str, id)
						self.dragRectObjs[i].set_title(textstr)
					
					self.update_message_boxes()
					self.tempInsertObjectData = dict()
					self.show_trajectory()
			elif not self.tempInsertObjectData:
				self.tempInsertObjectData = dict() 

		elif self.currHitTab =='Road Reconstruction':
			
			if self.SamplesFlag_N:
				self.RoadDataList_N.append(self.tempRoadData)
			elif self.SamplesFlag_W:
				self.RoadDataList_W.append(self.tempRoadData)
			elif self.SamplesFlag_S:
				self.RoadDataList_S.append(self.tempRoadData)
			elif self.SamplesFlag_E:
				self.RoadDataList_E.append(self.tempRoadData)
			#print(self.RoadDataList)
		
		
		
		
		'''
			pass #do nothing
		if need_update:
			self.update_message_boxes()  # update result message box
		return
		'''

	# Integrate following drags into mouse response events?
	def drag_oval_start(self, event):
		#only work on edition mode: 
		if not self.calAddEditMode: 
			self.tempDragData = dict()
			return
		self.tempDragData = dict()
		self.tempDragData["item"] = self.canvasPaintFrame.canvas.find_closest(event.x, event.y)[0]
		self.tempDragData["x"] = event.x
		self.tempDragData["y"] = event.y

	def drag_oval_stop(self, event, tag):
		if not bool(self.tempDragData):
			return
		if tag=='boundary_points': 
			if self.bound_oval_id: 
				for key in self.bound_oval_id: 
					if self.tempDragData["item"] == self.bound_oval_id[key]: 
						self.roadBoundPts[key][0] = self.tempDragData["x"]
						self.roadBoundPts[key][1] = self.tempDragData["y"]
		self.tempDragData["item"] = None
		self.tempDragData["x"] = 0
		self.tempDragData["y"] = 0
		self.update_message_boxes()

	def drag_oval(self, event, tag):
		if not bool(self.tempDragData):
			return
		delta_x = event.x - self.tempDragData["x"]
		delta_y = event.y - self.tempDragData["y"]
		self.canvasPaintFrame.canvas.move(self.tempDragData["item"], delta_x, delta_y)
		self.tempDragData["x"] = event.x
		self.tempDragData["y"] = event.y
	
	def cancel_bbox(self):
		self.canvasPaintFrame.canvas.delete("temp_oval")
		self.canvasPaintFrame.canvas.delete("temp_rect")
		self.tempInsertObjectData = dict()#clean temporay storage

	# --------Navigation-------------
	def reset_canvas(self): 
		self.canvasPaintFrame.reset_canvas()
		#rebind
		self.canvasPaintFrame.canvas.bind("<Button-1>", self.mouse_click)
		self.canvasPaintFrame.canvas.bind("<Motion>", self.mouse_move)#, "+")
		self.canvasPaintFrame.canvas.bind("<B1-Motion>", self.mouse_drag)
		self.canvasPaintFrame.canvas.bind("<ButtonRelease-1>", self.mouse_release)

	def goto(self):
		self.reset_canvas()
		self.load_image_data(self.kf_idx_list[self.currFrameId])
		self.clean_draw_objects()
		self.draw_image(self.imgDisp)
		self.draw_annotations()
		self.update_message_boxes()
		self.statusFrameLabel.config(text ='Frame {}/{}'.format(self.kf_idx_list[self.currFrameId], self.totalFrameNum))
		self.frameLabel.config(text ='Frame {}'.format(self.kf_idx_list[self.currFrameId]))
	
	def play_video(self): 
		self.currFrameId += 1
		self.currFrameId = max(0, min(self.totalFrameNum-1, self.currFrameId))
		self.goto()
		if not self.pauseMode: 
			self.videoNavigationFrame.after(30, self.play_video) #15
		if self.currFrameId == (self.totalFrameNum-1): #finished all play
			self.playBtn['text']='|>'
			self.pauseMode = True


	def play(self):
		if self.currFrameId is None: 
			return 
		if self.playBtn['text'] =='||': 
			self.pauseMode = True
		elif self.playBtn['text'] =='|>': 
			self.pauseMode = False
		if not self.pauseMode: 
			self.playBtn['text'] = '||' #change to pause
			#resume play 
			self.pauseMode = False
			self.play_video()
		else: 
			self.playBtn['text']='|>'
			self.pauseMode = True

	def goto_first(self):
		if self.currFrameId is None: 
			return 
		self.playBtn['text']='|>'
		self.pauseMode = True
		self.currFrameId = 0
		self.goto()

	def goto_prev(self):
		if self.currFrameId is None: 
			return 
		self.playBtn['text']='|>'
		self.pauseMode = True
		self.currFrameId = max(0, self.currFrameId-1)
		self.goto()

	def goto_prev10(self):
		if self.currFrameId is None: 
			return 
		self.playBtn['text']='|>'
		self.pauseMode = True
		self.currFrameId = max(0, self.currFrameId-10)
		self.goto()

	def goto_next(self):
		if self.currFrameId is None: 
			return 
		self.playBtn['text']='|>'
		self.pauseMode = True
		self.currFrameId = min(self.totalFrameNum-1, self.currFrameId+1)
		self.goto()

	def goto_next10(self):
		if self.currFrameId is None: 
			return 
		self.playBtn['text']='|>'
		self.pauseMode = True
		self.currFrameId = min(self.totalFrameNum-1, self.currFrameId+10)
		self.goto()

	def goto_last(self):
		if self.currFrameId is None: 
			return 
		self.playBtn['text']='|>'
		self.pauseMode = True
		self.currFrameId = self.totalFrameNum-1
		self.goto()
	
	def video_edit(self): 
		if self.videoCap is None: 
			self.videoFilename = filedialog.askopenfilename(title="Select Video", filetypes=(("video files", "*.mp4"), ("video files", "*.avi"),
																						 ("all files", "*.*")))
			if not self.videoFilename: 
				return
			self.videoCap = cv2.VideoCapture(self.videoFilename)
		
		ws = VideoClipperWindow(self.parent, self.videoCap)
		self.parent.wait_window(ws.top)
		if not ws.saver_filename is None: #reload new video  
			self.open_video(ws.saver_filename)


if __name__ == '__main__':
	root = Tk()
	
	# root.geometry('800x600')
	imgicon = PhotoImage(file='./icon/icon.gif')
	root.tk.call('wm', 'iconphoto', root._w, imgicon)
	windowWidth = 1200  # root.winfo_reqwidth()
	windowHeight = 760  # root.winfo_reqheight()
	positionRight = int(root.winfo_screenwidth()/2 - windowWidth/2)
	positionDown = int(root.winfo_screenheight()/2 - windowHeight/2)
	root.geometry("{}x{}+{}+{}".format(windowWidth,
									   windowHeight, positionRight, positionDown))
	args = parse_args()
	tool = MainGUI(root,args.videoname)
	#print(root.cget('bg'))
	root.mainloop()
