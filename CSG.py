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
from src.ui.canvas import ZoomFrame	 # , ZoomFrame2
from src.ui.image_utils import *
from src.ui.drag_line import DragLine
from src.calibration.calib_main import camera_clibration_canvas
from src.detect.detect_factory import Detecter
from src.track.track_factory import	 TRACKER
from src.track.trajectory import postprocessing_trajectory,select_trajectory, auto_refine
from src.road.road_main import initialize_bounary_points, generate_road
from src.error_check import multi_frame_error_check
from config import DET_CONFIG,TRACK_CONFIG, SOURCE
from src.xml_generation import convert2XML

'''
Note: 
All coordinates loaded in variables are canvas coordiantes(for drawing)
Saved coordinates are pixel-based, so need conversion when save/load

'''
CANVAS_WIDTH = 640
CANVAS_HEIGHT = 480
#CANVAS_WIDTH = 1400
#CANVAS_HEIGHT = 800
ORIENTATION_LIST =  ['North', 'West', 'South', 'East']
RADIUS = 7#to draw oval in road
LINEWIDTH = 2 #draw line
SMALL_CANVAS_SIZE=200

SHOW_ZOOMED_FOCUS = True

class MainGUI:
	def __init__(self, master):
		self.parent = master
		self.parent.title("CSG ToolKit v1.0(Survelliance)")

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
		self.titleLabel = Label(self.titleFrame, text="Video:",	 anchor=W)
		self.openBtn = Button(self.titleFrame, text="Open Video",command=self.open_video) 
		self.cameraViewBox = ttk.Combobox(
			self.titleFrame, state='readonly', textvar=self.cameraView, values=["Survelliance", "Ego-view"])
		self.cameraViewBox.current(0)

		self.titleLabel.pack(fill=X, side=LEFT, expand=1)
		self.cameraViewBox.pack(fill=X, side=RIGHT, padx=5, expand=0)
		self.openBtn.pack(fill=X, side=RIGHT, padx=5, expand=0)

		self.tabControl_canvas = ttk.Notebook(self.canvasFrame)
		self.canvasDisplayFrame = ttk.Frame(self.tabControl_canvas)
		#self.canvasDisplayPerspective = ttk.Frame(self.tabControl_canvas)
		self.tabControl_canvas.add(self.canvasDisplayFrame, text='Raw frame')
		#self.tabControl_canvas.add(self.canvasDisplayPerspective, text='Perspective view')
		self.tabControl_canvas.grid(row=0, column=0, columnspan=2)	# Pack to make visible

		self.videoNavigationFrame = ttk.Frame(self.canvasFrame, width=CANVAS_WIDTH-SMALL_CANVAS_SIZE-50)
		self.videoNavigationFrame.grid(row=1, column=0,padx=5, sticky="w")
		self.resultDisplayFrame = ttk.Frame(self.canvasFrame)
		self.resultDisplayFrame.grid(row=1, column=1, padx=10, pady=10, sticky="e")

		#self.canvasPaintPerspective = ZoomFrame(self.canvasDisplayPerspective, width=CANVAS_HEIGHT, height=CANVAS_HEIGHT,
		#								 background="gray")	 # cursor='tcross'

		self.canvasPaintFrame = ZoomFrame(self.canvasDisplayFrame, width=CANVAS_WIDTH, height=CANVAS_HEIGHT,
										 background="#90AFC5")	# cursor='tcross'
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
		self.tabDetect = ttk.Frame(self.tabControl)
		self.tabRoad = ttk.Frame(self.tabControl)
		self.tabScenario = ttk.Frame(self.tabControl)
		self.tabControl.add(self.tabCalib, text='Calibration')
		self.tabControl.add(self.tabDetect, text='Detect&Track')
		self.tabControl.add(self.tabRoad, text='Road Reconstruction')
		#self.tabControl.add(self.tabScenario, text='Scenario Lib')
		self.tabControl.pack(expand=1, fill="both")	 # Pack to make visible

		# tab-calibration
	   # self.calHlsBtn = Button(self.tabCalib, text='HLS_Image', command=self.show_hls)
	   # self.calHlsBtn.pack(fill=X, side=TOP, pady=5)
		self.calPointEditFrame = Frame(self.tabCalib)
		self.calPointEditFrame.pack(fill=X,pady=5, side=TOP)
		self.calAddPLBtn = Button(self.calPointEditFrame, text="Insert Parallel Line",command=self.select_parallel_line, 
								 disabledforeground='red')#fg="red", 
		self.calAddPLBtn.grid(row=0, column=0, sticky='we')
		self.calEditPLRPBtn = Button(self.calPointEditFrame, text="Adjust",command=self.select_edit_add,
								  disabledforeground = 'red')
		self.calEditPLRPBtn.grid(row=0, column=1, rowspan=2, sticky='ew')
		self.calAddRPBtn = Button(self.calPointEditFrame, text="Insert Ref Points",command=self.select_ref_points,
								  disabledforeground="red")
		self.calAddRPBtn.grid(row=1, column=0, sticky='we')
		self.calPointEditFrame.columnconfigure(0, weight=1)
		self.calPointEditFrame.columnconfigure(1, weight=1)
		Button(self.tabCalib, text="Re-Selection",command=self.clean_cal_selections).pack(fill=X, side=TOP)	 # , pady=1)
		Button(self.tabCalib, text="AutoCalibrate", command=self.cam_calib).pack(fill=X, side=TOP)	# , pady=1)
		Button(self.tabCalib, text="Save Results",command=self.calib_save_results).pack(fill=X, side=TOP)  # , pady=1)
		Button(self.tabCalib, text="Display Results",command=self.calib_disp_results).pack(fill=X, side=TOP)  # , pady=1)
		ttk.Separator(self.tabCalib, orient=HORIZONTAL).pack(fill=X, pady=10, side=TOP)	 # grid(..., sticky="ew")
		self.calFrameLbl = Label(self.tabCalib, text="")
		self.calFrameLbl.pack(fill=X, side=TOP)
		Label(self.tabCalib, text="Selected Parallel Lines").pack(fill=X, side=TOP)
		self.calPLListBox = Listbox(self.tabCalib, width=40, height=5)
		self.calPLListBox.pack(fill=X, side=TOP)
		Label(self.tabCalib, text="Selected Reference Lines").pack(fill=X, side=TOP)
		self.calRPListBox = Listbox(self.tabCalib, width=40, height=5)
		self.calRPListBox.pack(fill=X, side=TOP)

		self.calPoseAdjustFrame = Frame(self.tabCalib, borderwidth=1)
		Label(self.calPoseAdjustFrame, text="Pitch").grid( row=0, column=0, sticky='nesw')
		Label(self.calPoseAdjustFrame, text="Yaw").grid(row=0, column=2, sticky='nesw')
		Label(self.calPoseAdjustFrame, text="Roll").grid( row=0, column=4, sticky='nesw')
		self.calPitchScale = Scale(self.calPoseAdjustFrame, from_=-3.15, to=3.15,
								   resolution=0.01, orient=HORIZONTAL, command=self.adjust_camera_pose_value)
		self.calPitchScale.grid(row=1, column=0, sticky='nesw')
		self.calYawScale = Scale(self.calPoseAdjustFrame, from_=-3.15, to=3.15,
								 resolution=0.01, orient=HORIZONTAL, command=self.adjust_camera_pose_value)
		self.calYawScale.grid(row=1, column=2, sticky='nesw')
		self.calRollScale = Scale(self.calPoseAdjustFrame, from_=-3.15, to=3.15,
								  resolution=0.01, orient=HORIZONTAL, command=self.adjust_camera_pose_value)
		self.calRollScale.grid(row=1, column=4, sticky='nesw')
		ttk.Separator(self.calPoseAdjustFrame, orient=VERTICAL).grid(column=1, row=0, rowspan=2, sticky='ns')
		ttk.Separator(self.calPoseAdjustFrame, orient=VERTICAL).grid(column=3, row=0, rowspan=2, sticky='ns')

		self.calPoseAdjustFrame.rowconfigure(0, weight=1)
		self.calPoseAdjustFrame.columnconfigure(0, weight=1)  # .grid_columnconfigure(4, weight=1)
		self.calPoseAdjustFrame.columnconfigure(2, weight=1)
		self.calPoseAdjustFrame.columnconfigure(4, weight=1)
		self.calPoseAdjustFrame.pack(fill=X, side=TOP)
		Label(self.tabCalib, text="Calibration Results").pack(fill=X, side=TOP)
		self.calRstListBox = Listbox(self.tabCalib, height=20)
		self.calRstListBox.pack(fill=X, side=TOP)

		# tab-detection
		Label(self.tabDetect, text='Select Detect Model').pack(fill=X, side=TOP)
		self.detModelBox = ttk.Combobox(
			self.tabDetect, state='readonly', textvar=self.detectionModel, values=["YOLO v4", "Mask RCNN"])
		self.detModelBox.pack(fill=X, side=TOP)
		self.detModelBox.current(0)
		Button(self.tabDetect, text="Auto Annotation !", command=self.auto_annotation).pack(fill=X,pady=10, side=TOP)
		Button(self.tabDetect, text="Trajectory Interpolation", command=self.traj_interp).pack(fill=X, side=TOP)
		Button(self.tabDetect, text="Reset Canvas", command=self.reset_canvas).pack(fill=X, side=TOP)
		Button(self.tabDetect, text="Save", command=self.save_anno_results).pack(fill=X, side=TOP)
		ttk.Separator(self.tabDetect, orient=HORIZONTAL).pack(fill=BOTH, pady=20, side=TOP)

		self.detObjectsEditFrame = Frame(self.tabDetect)
		Label(self.detObjectsEditFrame, text="List of Objects").grid(row=0, column=0, columnspan=2, sticky='we')
		self.detTrajShowCheckbox = Checkbutton(self.detObjectsEditFrame, variable=self.enableTrajDisp, 
								onvalue=1, offvalue=0, text="Display Trajectory", command=self.show_trajectory)
		self.detTrajShowCheckbox.grid(row=0, column=2, sticky='ew')
		self.detObjectListBox = Listbox(self.detObjectsEditFrame)
		self.detObjectListBox.grid(row=1, column=0, columnspan=3, sticky='we')
		self.insertObjectBtn = Button(self.detObjectsEditFrame, text="Add", command=self.insert_object, 
									  disabledforeground='red')
		self.insertObjectBtn.grid(row=2, column=0, sticky='nesw')
		self.modifyObjectBtn = Button(self.detObjectsEditFrame, text="Edit", command=self.modify_object,
									  disabledforeground='red')
		self.modifyObjectBtn.grid(row=2, column=1, sticky='nesw')
		Button(self.detObjectsEditFrame, text='Delete', command=self.delete_object).grid(row=2, column=2, sticky='nesw')
		self.detObjectsEditFrame.rowconfigure(0, weight=1)
		self.detObjectsEditFrame.columnconfigure(0, weight=1)
		self.detObjectsEditFrame.columnconfigure(1, weight=1)
		self.detObjectsEditFrame.columnconfigure(2, weight=1)
		self.detObjectsEditFrame.pack(fill=X, side=TOP)

		ttk.Separator(self.tabDetect, orient=HORIZONTAL).pack(fill=BOTH, pady=20, side=TOP)
		Label(self.tabDetect, text="List of Frames").pack(fill=X, side=TOP)
		self.viewFrame = Frame(self.tabDetect)
		frameTreeScroll = ttk.Scrollbar(self.viewFrame,orient='vertical')
		frameTreeScroll.grid(row=0, column=1, sticky='ns')
		self.frameTreeView = ttk.Treeview(self.viewFrame,columns=2)#, show=['headings'])
		self.frameTreeView.grid(row=0, column=0, sticky='nswe')
		frameTreeScroll.configure(command=self.frameTreeView.yview)
		self.frameTreeView.configure(yscrollcommand=frameTreeScroll.set)
		self.viewFrame.columnconfigure(0, weight=1)
		self.viewFrame.pack(side=TOP, fill=X)
		# tab-road
	   ## self.roadEnhanceBtn = Button(self.tabRoad, text='Enhanced Image', command=self.show_enhanced)
	   # self.roadEnhanceBtn.pack(fill=X, side=TOP)

		self.roadBoundaryBtn = Button(self.tabRoad, text="Initialize Boundary Points", command=self.init_bound_points)
		self.roadBoundaryBtn.pack(fill=X, pady=10, side=TOP)
		Label(self.tabRoad, text='Adjust boundary points by dragging').pack(fill=X, side=TOP)
		Label(self.tabRoad, text='Blue: North boundary point',fg=marker_colors['North']).pack(fill=X, side=TOP)
		Label(self.tabRoad, text='Yellow: West boundary point',fg=marker_colors['West']).pack(fill=X, side=TOP)
		Label(self.tabRoad, text='Green: South boundary point',fg=marker_colors['South']).pack(fill=X, side=TOP)
		Label(self.tabRoad, text='Red: East boundary point',fg=marker_colors['East']).pack(fill=X, side=TOP)

		NorthRoadSampleBtn = Button(self.tabRoad, text="Select North Sample Points", command=self.select_sample_pts_n,fg=marker_colors['North'])
		NorthRoadSampleBtn.pack(fill=X, pady=1, side=TOP)
		WestRoadSampleBtn = Button(self.tabRoad, text="Select West Sample Points", command=self.select_sample_pts_w,fg=marker_colors['West'])
		WestRoadSampleBtn.pack(fill=X, pady=1, side=TOP)
		SouthRoadSampleBtn = Button(self.tabRoad, text="Select South Sample Points", command=self.select_sample_pts_s,fg=marker_colors['South'])
		SouthRoadSampleBtn.pack(fill=X, pady=1, side=TOP)
		EastRoadSampleBtn = Button(self.tabRoad, text="Select East Sample Points", command=self.select_sample_pts_e,fg=marker_colors['East'])
		EastRoadSampleBtn.pack(fill=X, pady=1, side=TOP)

		ttk.Separator(self.tabRoad, orient=HORIZONTAL).pack(fill=X, pady=20, side=TOP)
		Label(self.tabRoad, text="Lane Number").pack(fill=X, side=TOP)
		road_icon_image = cv2.imread('icon/intersection.jpg')
		self.laneSettingFrame = LaneSettingFrame(self.tabRoad, road_icon_image, width=300, height=300)
		self.laneSettingFrame.pack(fill=X, side=TOP)

		self.roadGenBtn = Button(self.tabRoad, text="Generate Road", command=self.generate_road_net)  # self.GenerateRoadNet
		self.roadGenBtn.pack(fill=X, pady=5, side=TOP)

		# tab-scenario-lib
		Label(self.tabScenario, text='Critical Scenario Lib is generated from real traffic accident videos.',wraplength=300, font=("Helvetica", 12)).pack(fill=BOTH, expand=1)
		Button(self.tabScenario, text="View Critical Scenario Lib", command=self.start_sim_lib,
			   font=("Helvetica", 16), background='blue', fg='white',
			   highlightbackground="white", highlightcolor="white",
				highlightthickness=1, anchor="center").pack(fill=X, expand=1)

		# status frame
		self.statusLabel = Label(self.statusFrame, text="Status:", bd=1, relief=SUNKEN, anchor=W)
		self.statusLabel.pack(fill=X, side=LEFT, expand=True)
		self.statusFrameLabel = Label(self.statusLabel, text="Frame", bd=1, relief=SUNKEN, anchor=E)
		self.statusFrameLabel.pack(fill=BOTH, side=RIGHT, expand=False)

		self.parent.grid_rowconfigure(1, weight=1)
		self.parent.grid_columnconfigure(1, weight=1)
		self.bind_events()



		# ----------------- Variables---------------------
		self.videoFilename = None
		self.videoCap = None
		self.totalFrameNum = 0
		self.currFrameId = None
		self.calFrameID = None	# frame id to do clibration
		self.roadFrameID = None	 # frame id to do road reconstruction
		self.imgRaw = None	# cv mode(bgr)
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
		self.imgPerspective = None	# cv mode(bgr)

		self.calParaLines = []	# canvas_coords,group of 4 points
		self.calRefPoints = []	# canvas_coords,group of 2 points
		self.calParaLinesObjIds = []
		self.calRefPointsObjIds = []
		self.cameraParam = None
		self.roadBoundPts = dict()	# save selected roudn poits
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

	def bind_events(self):	# bind all messages
		self.parent.bind("<Key-Left>", self.goto_prev)
		self.parent.bind("<Key-Right>", self.goto_next)
		self.parent.bind("Escape", self.cancel_bbox)

		self.tabControl.bind("<<NotebookTabChanged>>", self.tab_click)

		self.canvasPaintFrame.canvas.bind("<Button-1>", self.mouse_click)
		self.canvasPaintFrame.canvas.bind("<Motion>", self.mouse_move)#, "+")
		self.canvasPaintFrame.canvas.bind("<B1-Motion>", self.mouse_drag)
		self.canvasPaintFrame.canvas.bind("<ButtonRelease-1>", self.mouse_release)

		#self.canvasPaintPerspective.canvas.bind("<Motion>", self.mouse_moveP)#, "+")

		self.calPLListBox.bind('<Key>', self.edit_parallel_line_keypress)
		self.calRPListBox.bind('<Double-Button>', self.edit_reference_points)
		self.calRPListBox.bind('<Key>', self.edit_reference_points_keypress)
		
		self.detObjectListBox.bind('<Double-Button>', self.edit_object_label_id)
		self.detObjectListBox.bind('<<ListboxSelect>>', self.listbox_select)
		self.detObjectListBox.bind('<Key>', self.object_listbox_keypress)

		self.frameTreeView.bind('<Double-Button>', self.treeview_click)
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
		self.canvasPaintFrame.canvas.delete('all')
		self.reset_modes(True)
		self.totalFrameNum = int(self.videoCap.get(7))
		self.currFrameId = 0
		self.titleLabel.config(text="Video Path: "+self.videoFilename)
		self.load_image_data(self.currFrameId)
		self.load_calib_results()
		self.load_anno_results()
		self.load_road_results()

		self.draw_image(self.imgDisp)
		self.draw_annotations()
		self.generate_thumbnail_view()
		self.update_message_boxes()
		self.statusFrameLabel.config(text ='Frame {}/{}'.format(self.currFrameId+1, self.totalFrameNum))
		self.frameLabel.config(text ='Frame {}'.format(self.currFrameId+1))

	def load_image_data(self, frame_id):
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
		elif self.currHitTab=='Calibration':
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

	def adjust_camera_pose_value(self, event):
		if self.cameraParam is None:
			return
		pitch_input = self.calPitchScale.get()
		roll_input = self.calRollScale.get()
		yaw_input = self.calYawScale.get()
		self.cameraParam.Pitch = pitch_input
		self.cameraParam.Roll = roll_input
		self.cameraParam.Yaw = yaw_input
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
						info_str += '	'
					if SOURCE['Track'] in src_list: 
						info_str += ' T '
					else: 
						info_str += '	'
					if SOURCE['Manual'] in src_list: 
						info_str += ' M '
					else: 
						info_str += '	'
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
		elif self.currHitTab=='Calibration':
			# fill in calParaLines->calPLListBox
			self.calPLListBox.delete(0, END)
			# each paraline has 4 points
			for lines in self.calParaLines:
				temp_str = '{}:'.format(lines[0][0])
				for j in range(2):
					temp_str += '({},{})-({},{})'.format(lines[j][1],lines[j][2],
												 lines[j][3],lines[j][4])
					if j==0: 
						temp_str +='\t'
				self.calPLListBox.insert(END, temp_str)
			# fill in calRefPoints ->calRPListBox
			self.calRPListBox.delete(0, END)
			for i, line in enumerate(self.calRefPoints):
				temp_str = '{}: ({},{})-({},{})\t Distance={}'.format(
					line[0], line[1], line[2], line[3], line[4], line[5])
				self.calRPListBox.insert(END, temp_str)
			# todo: fill result pose
			if not self.cameraParam is None:
				self.calRstListBox.delete(0, END)
				self.calRstListBox.insert(END, "Fx:	 " + str(self.cameraParam.Fx))
				self.calRstListBox.insert(END, "Fy:	 " + str(self.cameraParam.Fy))
				self.calRstListBox.insert(END, "Cx:	 " + str(self.cameraParam.Cx))
				self.calRstListBox.insert(END, "Cy:	 " + str(self.cameraParam.Cy))
				self.calRstListBox.insert(END, "Pitch:	" + str(self.cameraParam.Pitch))
				self.calRstListBox.insert(END, "Roll:  " + str(self.cameraParam.Roll))
				self.calRstListBox.insert(END, "Yaw:  " + str(self.cameraParam.Yaw))
				self.calRstListBox.insert(END, "Tx:	 " + str(self.cameraParam.Tx))
				self.calRstListBox.insert(END, "Tz:	 " + str(self.cameraParam.Tz))
				self.calRollScale.set(self.cameraParam.Roll)
				self.calPitchScale.set(self.cameraParam.Pitch)
				self.calYawScale.set(self.cameraParam.Yaw)
			else: 
				self.calRstListBox.delete(0, END)
				
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
		self.calAddPLBtn['state'] = 'normal'
		self.calEditPLRPBtn['state'] = 'disable'
		self.calAddRPBtn['state'] = 'normal'
		self.pauseMode = False 
		if self.ObjectList is None and self.totalFrameNum>0: #if not initialize
			self.ObjectList = [[] for _ in range(self.totalFrameNum)]
			self.ObjectList_pixel = [[] for _ in range(self.totalFrameNum)]
		self.InsertObjectMode = False
		self.insertObjectBtn['state'] = 'normal'
		self.modifyObjectBtn['state'] = 'disable'
		#default button color 
		self.calEditPLRPBtn.configure(bg='#6495ed')
		self.modifyObjectBtn.configure(bg='#6495ed')
		c = self.parent.cget('bg')
		self.calAddRPBtn.configure(bg=c)
		self.calAddPLBtn.configure(bg=c)
		self.insertObjectBtn.configure(bg=c)

		if clean_results: 
			self.ObjectList = None
			self.ObjectList_pixel = None
			self.cameraParam = None
			self.roadBoundPts = dict()
			self.roadLaneNum = {'ne':1, 'nw':1, 'wn':1, 'ws':1, 'sw':1, 'se':1,'es':1,'en':1}#default
			self.calParaLines = []	# canvas_coords,group of 4 points
			self.calRefPoints = []	# canvas_coords,group of 2 points
			self.calParaLinesObjIds = []
			self.calRefPointsObjIds = []
			self.trajectory = None
			


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
		  #	 self.calHlsBtn["text"] = "HLS Image"
			self.draw_image(self.imgDisp)
			self.draw_annotations()
	
	def select_edit_add(self): 
		c = self.parent.cget('bg')
		self.calAddEditMode = not self.calAddEditMode
		if not self.calAddEditMode: 
			self.calEditPLRPBtn['state']='normal'
			self.calAddPLBtn['state'] = 'disable'
			self.calAddRPBtn['state'] = 'disable'
		else: 
			self.calEditPLRPBtn['state']='disable'
			self.calAddRPBtn['state']='normal'
			self.calAddPLBtn['state']='normal'
			self.calEditPLRPBtn.configure(bg='#6495ed')
			self.calAddPLBtn.configure(bg=c)
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
		self.calAddPLBtn['state'] = 'normal'
		self.calEditPLRPBtn['state'] = 'normal'
		self.calAddRPBtn['state']='disable'	   
		self.calAddRPBtn.configure(bg='#6495ed')
		self.calEditPLRPBtn.configure(bg=c)
		self.calAddPLBtn.configure(bg=c)

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
	def clean_cal_selections(self):	 # re-select
		self.clean_draw_objects()
		self.calParaLines = []
		self.calRefPoints = []
		self.tempPLLine = []
		self.tempDragData = None  # drag boundary points
		self.update_message_boxes()
		return

	def cam_calib(self):
		if len(self.calParaLines) < 2:
			messagebox.showinfo(
				"Info", "Please select at least 2 parallel lines")
			return
		if len(self.calRefPoints) < 1:
			messagebox.showinfo(
				"Info", "Please select at least 1 reference line")
			return
		self.cameraParam = camera_clibration_canvas(self.calParaLines, self.calRefPoints,
													(self.imgRaw.shape[1],
													 self.imgRaw.shape[0]),
													self.imgScale, self.imgPadding)
		self.update_message_boxes()

	def calib_save_results(self):
		txt_saver = self.videoFilename+'_camera_calibration_result.txt'
		if not self.cameraParam is None: 
			with open(txt_saver, 'w+') as f:
				f.write('Fx: ' + str(self.cameraParam.Fx) + '\n')
				f.write('Fy: ' + str(self.cameraParam.Fy) + '\n')
				f.write('Cx: ' + str(self.cameraParam.Cx) + '\n')
				f.write('Cy: ' + str(self.cameraParam.Cy) + '\n')
				f.write('Pitch: ' + str(self.cameraParam.Pitch) + '\n')
				f.write('Roll: ' + str(self.cameraParam.Roll) + '\n')
				f.write('Yaw: ' + str(self.cameraParam.Yaw) + '\n')
				f.write('Tx: ' + str(self.cameraParam.Tx) + '\n')
				f.write('Ty: ' + str(self.cameraParam.Ty) + '\n')
				f.write('Tz: ' + str(self.cameraParam.Tz) + '\n')
				f.write('Mat_K: ' + str(self.cameraParam.Mat_K) + '\n')
				f.write('Mat_R: ' + str(self.cameraParam.Mat_R) + '\n')
				f.write('Mat_P: ' + str(self.cameraParam.Mat_P) + '\n')
		pkl_saver = self.videoFilename+'_camcalib.pkl'
		calParaLine_pixel = copy.deepcopy(self.calParaLines)
		for i, item in enumerate(calParaLine_pixel): 
			for j, line in enumerate(item): 
				canvas_coords=[[line[1], line[2],line[3], line[4]]]
				pixel_coords = convert_canvas_to_pixel(canvas_coords, self.imgScale, self.imgPadding)[0]
				calParaLine_pixel[i][j][1] = pixel_coords[0]
				calParaLine_pixel[i][j][2] = pixel_coords[1]
				calParaLine_pixel[i][j][3] = pixel_coords[2]
				calParaLine_pixel[i][j][4] = pixel_coords[3]
		calRefPoints_pixel = copy.deepcopy(self.calRefPoints)
		for i, item in	enumerate(calRefPoints_pixel): 
			canvas_coords=[[item[1], item[2], item[3], item[4]]]
			pixel_coords = convert_canvas_to_pixel(canvas_coords, self.imgScale, self.imgPadding)[0]
			calRefPoints_pixel[i][1] = pixel_coords[0]
			calRefPoints_pixel[i][2] = pixel_coords[1]
			calRefPoints_pixel[i][3] = pixel_coords[2]
			calRefPoints_pixel[i][4] = pixel_coords[3]
		with open(pkl_saver, 'wb') as f:
			pickle.dump([self.calFrameID, calParaLine_pixel,
						 calRefPoints_pixel, self.cameraParam], f)

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
		for i, item in	enumerate(refpts_pixels): 
			pixel_coords=[[item[1], item[2], item[3], item[4]]]
			canvas_coords = convert_pixel_to_canvas(pixel_coords, self.imgScale, self.imgPadding)[0]
			self.calRefPoints[i][1] = canvas_coords[0]
			self.calRefPoints[i][2] = canvas_coords[1]
			self.calRefPoints[i][3] = canvas_coords[2]
			self.calRefPoints[i][4] = canvas_coords[3]
			

	#Display the pespective view for fine tuning
	def calib_disp_results(self, canvas_coord=None):
		if self.imgRaw is None: 
			return
		self.animationDisplayCanvas.delete('all')

		if self.cameraParam is None:
			return 
		fpitch = self.calPitchScale.get()
		froll = self.calRollScale.get()
		fyaw = self.calYawScale.get()

		#note the perspective image is a square image, different resolution with raw image
		print("start calc perspective view")
		self.imgPerspective, self.cameraParm = convert_perspective_image(self.imgRaw, 
									froll, fpitch, fyaw, self.cameraParam,
									canvas_size=CANVAS_HEIGHT)
		#self.imgDisp, _, self.imgScale, self.imgPadding = fit_image_to_canvas(self.imgRaw,
		#								 desired_width=CANVAS_WIDTH, desired_height=CANVAS_HEIGHT)
		self.imgPerspectiveTk = ImageTk.PhotoImage(PILImage.fromarray(self.imgPerspective))
		print("finish calc perspective view")
			
		#self.canvasPaintPerspective.load_image(PILImage.fromarray(self.imgPerspective))


	#Display the zoomed perspective 
	def show_zoomed_perspective(self, canvas_coord=None):
		if self.imgPerspective is None: 
			print("self.imgPerspective is None")
			return
		#self.animationDisplayCanvas.delete('all')
		if SHOW_ZOOMED_FOCUS: #show cropped enlarged image, center on mouse
			try: 
				pixel_coord = convert_canvas_to_pixel([canvas_coord], self.imgScale, self.imgPadding)[0]
				raw_img_arr =  copy.deepcopy(self.imgPerspective)
				#raw_img_pil = Image.fromarray(cv2.cvtColor(self.imgRaw, cv2.COLOR_BGR2RGB))
				raw_img_pil = PILImage.fromarray(raw_img_arr)
				x,y = pixel_coord[0], pixel_coord[1]
				print("[x, y] is", x, y)
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

	#Display the zoomed focus 
	def show_zoomed_focus(self, canvas_coord=None):
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


	# --------tab2:Detection-----------

	def auto_annotation(self):
		#init model 
		if self.imgRaw is None: 
			return
		if any(self.ObjectList): 
			msgbox = messagebox.askquestion("Notice", "AutoRun with clean all current annotaions. Are you sure you want to continue?",icon='warning')
			if not msgbox =='yes': 
				return 
		model_name = self.detectionModel.get()
		view_name = self.cameraView.get()
		popup_msg =Toplevel()#,takefocus=True)
		x = self.parent.winfo_x()
		y = self.parent.winfo_y()
		popup_msg.geometry("%dx%d+%d+%d" % (400,80, x + 400, y + 200))
		popup_msg.grab_set()
		popup_msg.resizable(False, False)
		popup_msg_lbl = Label(popup_msg, text="Loading detection and tracking models")
		popup_msg_lbl.pack()#grid(row=0,column=0)
		progress_var = DoubleVar()
		progress_bar = ttk.Progressbar(popup_msg, variable=progress_var, maximum=100,
									orient=HORIZONTAL, length=350)#, mode="indeterminate")
		progress_bar.pack(side=TOP)
		popup_msg.pack_slaves()
		progress_var.set(20)
		popup_msg.update()
		#step1: loading model
		try: #loading detection model 
			if self.detector is None: 
				self.detector = Detecter()
				self.detector.load_model(model_name, view_name,self.labelList, DET_CONFIG)
			elif self.detector.model_name != model_name or self.detector.view_name != view_name: 
				self.detector.load_model(model_name, view_name, self.labelList, DET_CONFIG)
			#else do not need to reload_model
		except Exception as e: #loading tracking model 
			popup_msg.destroy()
			self.statusLabel.config(text ='Load detection model failed with error={}'.format(e))
			return
		
		progress_var.set(70)
		popup_msg.update()
		try:
			if self.tracker is None: 
				self.tracker = TRACKER(TRACK_CONFIG)
			else: 
				self.tracker.reset()
		except Exception as e: 
			popup_msg.destroy()
			self.statusLabel.config(text ='Load tracking model failed with error={}'.format(e))
			return
		
		#step 2: loop 
		#Object_list:list of [frame_data], each framedata: list of [object_data]
		# object_data: (object_id, object_label_id, x1,y1,x2,y2, score, source)
		self.ObjectList = [[] for _ in range(self.totalFrameNum)]
		self.ObjectList_pixel = [[] for _ in range(self.totalFrameNum)]
		self.nextObjectID = 1
		self.statusLabel.config(text ='Auto detect and track objects')
		self.reset_canvas()
		self.goto_first()
		
		for frame_id in range(self.totalFrameNum): 
			progress_var.set(round((frame_id/self.totalFrameNum)*100))
			popup_msg_lbl.config(text='Processing: {}/{}'.format(frame_id+1, self.totalFrameNum))
			popup_msg.update()

			self.load_image_data(frame_id)
			#Note : detection/tracking works in pixel coordinates in raw iamge
			#detection return: (object_label_id, x1,y1,x2,y2,score)
			#track return: (trk_id, object_label_id, score, x1,y1,x2,y2)
			#final output: (trk_id, object_label_id, x1,y1,x2,y2, score, label_source="Auto")
			curr_det_rst,_ =self.detector.detect(self.imgRaw)

			self.tracker.update(curr_det_rst, self.imgRaw, frame_id)
			curr_tracklets = self.tracker.get_current_result()
		
			curr_objs = []
			curr_objs_pixel = []
			for tr in curr_tracklets: 
				lt = convert_pixel_to_canvas([[tr[3], tr[4]]], self.imgScale, self.imgPadding)[0]
				rb =  convert_pixel_to_canvas([[tr[5], tr[6]]], self.imgScale, self.imgPadding)[0] 
				source = SOURCE['Auto'] #from tracking 
				curr_objs.append([tr[0], tr[1], lt[0], lt[1], rb[0], rb[1], tr[2], source]) #reorder
				curr_objs_pixel.append([tr[0], tr[1], tr[3], tr[4], tr[5], tr[6], tr[2], source])
			self.ObjectList[frame_id] = curr_objs
			self.ObjectList_pixel[frame_id] = curr_objs_pixel 
		   # self.clean_draw_objects()
		   # self.draw_annotations()
		popup_msg_lbl.config(text='Wait for generating trajectories.')
  
		#traj format: list of object_id: [[object_id, frame_id, center_x,center_y]] ->num_framesx4
		self.trajectory, self.ObjectList, self.ObjectList_pixel, self.nextObjectID = \
				postprocessing_trajectory(self.videoCap,self.tracker,
										 self.ObjectList, self.ObjectList_pixel, 
										 self.imgScale, self.imgPadding)
		
		self.nextObjectID += 1
		self.goto_first()
		popup_msg.destroy()
		self.statusLabel.config(text ='Auto annotation done!')
	
	def traj_interp(self): 
		self.trajectory, self.ObjecList, self.ObjectList_pixel, _ = \
				postprocessing_trajectory(self.videoCap,self.tracker,
										 self.ObjectList, self.ObjectList_pixel, 
										 self.imgScale, self.imgPadding,
										 do_backward=False,do_reindex=False)
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
			

	def save_anno_results(self):
		saver = self.videoFilename+"_annotation.pkl"
		traj_pixel = []
		if not self.trajectory is None: 
			for traj in self.trajectory: 
				pt_arr = traj[:,2:]
				pt_1d = pt_arr.ravel()
				pixels = convert_canvas_to_pixel([list(pt_1d)], self.imgScale, self.imgPadding)[0]
				pixels_arr = np.reshape(np.asarray(pixels), (-1,2))
				traj_pixel.append(np.hstack((traj[:,:2], pixels_arr)))
		try: 
			with open(saver, 'wb') as f: 
				pickle.dump([self.ObjectList, self.ObjectList_pixel, traj_pixel, self.nextObjectID],f)
		except Exception as e: 
			 print('Failed to write detection results.')
			
		convert2XML(self.videoFilename)

	def load_anno_results(self): 

		saver = os.path.join(self.videoFilename+"_annotation.pkl")
		if not os.path.exists(saver): 
			return
		raw = pickle.load(open(saver, 'rb'))
		self.ObjectList = raw[0]
		self.ObjectList_pixel = raw[1]
		traj_pixel = raw[2]
		self.nextObjectID = raw[3]
		if len(traj_pixel)<=0: 
			self.trajectory = None
		else: 
			self.trajectory = []
			for traj in traj_pixel: 
				pixel_arr = traj[:, 2:]
				pixel_1d = pixel_arr.ravel()
				canvas = convert_pixel_to_canvas([list(pixel_1d)],self.imgScale, self.imgPadding)[0]
				canvas_arr = np.reshape(np.asarray(canvas), (-1,2))
				self.trajectory.append(np.hstack((traj[:,:2], canvas_arr)))
		#objectList should be recal based on current padding 
		for fid, frame in enumerate(self.ObjectList_pixel): 
			for aid, pix_anno in enumerate(frame): 
				pt1 = convert_pixel_to_canvas([[pix_anno[2], pix_anno[3]]], self.imgScale, self.imgPadding)
				pt2 = convert_pixel_to_canvas([[pix_anno[4], pix_anno[5]]], self.imgScale, self.imgPadding)
				self.ObjectList[fid][aid][2] = pt1[0][0]
				self.ObjectList[fid][aid][3] = pt1[0][1]
				self.ObjectList[fid][aid][4] = pt2[0][0]
				self.ObjectList[fid][aid][5] = pt2[0][1]

	def insert_object(self): 
		self.InsertObjectMode = True
		self.insertObjectBtn['state'] = 'disable'
		self.modifyObjectBtn['state'] = 'normal'
		self.insertObjectBtn.configure(bg='#6495ed')
		self.modifyObjectBtn.configure(bg=self.parent.cget('bg'))
		#lock all current object
		if not self.dragRectObjs is None: 
			for target in self.dragRectObjs: 
				target.freeze(True)
		self.statusLabel.config(text = "Mode: insert object")

	
	def modify_object(self): 
		self.InsertObjectMode = False
		self.insertObjectBtn['state'] = 'normal'
		self.modifyObjectBtn['state'] = 'disable'
		self.modifyObjectBtn.configure(bg='#6495ed')
		self.insertObjectBtn.configure(bg=self.parent.cget('bg'))
		
		#unlock all current object
		if not self.dragRectObjs is None: 
			for target in self.dragRectObjs: 
				target.freeze(False)
		self.statusLabel.config(text = "Mode: edit object")
	def delete_object(self): 
		selection = self.detObjectListBox.curselection()
		if len(selection)>0: 
			old_object_item = self.ObjectList[self.currFrameId][selection[0]]
			self.ObjectList, self.ObjectList_pixel, self.trajectory,_=\
					auto_refine(self.videoCap, self.parent, 
							self.currFrameId, None, 
							self.ObjectList, self.ObjectList_pixel, self.trajectory, 
							self.imgScale, self.imgPadding, 
							'delete', old_object_item)
			self.dragRectObjs[selection[0]].erase()
			del self.dragRectObjs[selection[0]]
			self.show_trajectory(False)
			self.show_trajectory(True)
			self.update_message_boxes()

 
	def object_listbox_keypress(self, event): 
		c = repr(event.char)
		if c == "'\\x7f'":	#  press 'delete'
			widget = event.widget
			selection = widget.curselection()
			if len(selection)>0: 
				old_object_item = self.ObjectList[self.currFrameId][selection[0]]
				self.ObjectList, self.ObjectList_pixel, self.trajectory,_=\
						auto_refine(self.videoCap, self.parent, 
								self.currFrameId, None, 
								self.ObjectList, self.ObjectList_pixel, self.trajectory, 
								self.imgScale, self.imgPadding, 
								'delete', old_object_item)
				self.dragRectObjs[selection[0]].erase()
				del self.dragRectObjs[selection[0]]
				self.show_trajectory(False)
				self.show_trajectory(True)
				self.update_message_boxes()
				
	def listbox_select(self, event): 
		widget = event.widget
		selection = widget.curselection()
		
		for i in range(len(self.dragRectObjs)): 
			if len(selection)<=0: 
				self.dragRectObjs[i].highlight(False)
			else: 
				if i==selection[0]:
					self.dragRectObjs[selection[0]].highlight(True)
				else: 
					self.dragRectObjs[i].highlight(False)

	def edit_object_label_id(self, event): 
		widget = event.widget
		selection = widget.curselection()
		old_object_item = self.ObjectList[self.currFrameId][selection[0]]
		old_id = old_object_item[0]	   
		old_label = self.labelList[old_object_item[1]]
		w = View_AskIDLabel(self.parent, self.labelList, ObjectID=old_id, label=old_label)	# canvasPaintFrame)
		self.parent.wait_window(w.top)
		if not w.label is None:
			new_object_item = copy.deepcopy(old_object_item)
			new_id = w.ObjectID
			new_label = w.label
			new_object_item[0] = new_id
			new_object_item[1] = self.labelList.index(new_label)
			self.ObjectList, self.ObjectList_pixel, self.trajectory,_=\
				auto_refine(self.videoCap, self.parent, 
						self.currFrameId, new_object_item, 
						self.ObjectList, self.ObjectList_pixel, self.trajectory, 
						self.imgScale, self.imgPadding, 
						'modify', old_object_item)
			textstr = "{}  {}".format(w.label, w.ObjectID)
			self.dragRectObjs[selection[0]].set_title(textstr)
			self.update_message_boxes()
	

	# --------tab3:-Road Reconstruction-----------
	def show_enhanced(self):#not used 
		if self.imgRaw is None:
			return
		self.clean_draw_objects()
		self.roadImageMode = not self.roadImageMode
		if self.roadImageMode:
		   # self.roadEnhanceBtn['text'] = 'Original Image'
			self.imgRoad = convert_road_enhance_image(self.imgRaw)
			self.imgRoadDisp = resize_padding_image(
				self.imgRoad, self.imgScale, self.imgPadding)
			self.draw_image(self.imgRoadDisp)
		else:
		  #	 self.roadEnhanceBtn['text'] = 'Enhanced Image'
			self.draw_image(self.imgDisp)
			self.draw_annotations()

	def select_sample_pts_n(self):
		if self.imgRaw is None: 
			return
		#c = self.parent.cget('bg')
		self.SamplesFlag_N = True
		self.SamplesFlag_W = False
		self.SamplesFlag_S = False
		self.SamplesFlag_E = False
		self.statusLabel.config(text = "Mode: North Sample Pts Select")
		
	def select_sample_pts_w(self):
		if self.imgRaw is None: 
			return
		#c = self.parent.cget('bg')
		self.SamplesFlag_N = False
		self.SamplesFlag_W = True
		self.SamplesFlag_S = False
		self.SamplesFlag_E = False
		self.statusLabel.config(text = "Mode: West Sample Pts Select")
		
	def select_sample_pts_s(self):
		if self.imgRaw is None: 
			return
		#c = self.parent.cget('bg')
		self.SamplesFlag_N = False
		self.SamplesFlag_W = False
		self.SamplesFlag_S = True
		self.SamplesFlag_E = False
		self.statusLabel.config(text = "Mode: South Sample Pts Select")
		
	def select_sample_pts_e(self):
		if self.imgRaw is None: 
			return
		#c = self.parent.cget('bg')
		self.SamplesFlag_N = False
		self.SamplesFlag_W = False
		self.SamplesFlag_S = False
		self.SamplesFlag_E = True
		self.statusLabel.config(text = "Mode: East Sample Pts Select")


	def init_bound_points(self):
		if self.imgRaw is None:
			return
		if bool(self.roadBoundPts):
			messagebox.showinfo(
				"Info", "Please adjust boundary points by dragging points")
			return
		self.roadFrameID = self.currFrameId
		pts = initialize_bounary_points(self.imgRaw, self.imgScale, self.imgPadding)
		self.bound_oval_id=dict()
		for o in ORIENTATION_LIST : 
			x,y = self.canvasPaintFrame.canvas.canvasx(pts[o][0]),self.canvasPaintFrame.canvas.canvasx(pts[o][1])
			self.bound_oval_id[o] = self.canvasPaintFrame.canvas.create_oval(x-RADIUS, y-RADIUS,
																		x+RADIUS, y+RADIUS,
																	  fill=marker_colors[o], tags='boundary_points')
		self.roadBoundPts = pts

		self.canvasPaintFrame.canvas.tag_bind("boundary_points", "<ButtonPress-1>", self.drag_oval_start)
		self.canvasPaintFrame.canvas.tag_bind("boundary_points", "<ButtonRelease-1>", lambda event, tag='boundary_points':self.drag_oval_stop(event, tag))
		self.canvasPaintFrame.canvas.tag_bind("boundary_points", "<B1-Motion>", lambda event, tag='boundary_points':self.drag_oval(event, tag))
 
	def generate_road_net(self):
		self.roadFrameID = self.currFrameId
		# check if all points have been selected
		if not bool(self.roadBoundPts):
			messagebox.showinfo(
				title='Warning', message='Please Setting boundary points first')
			return
		if self.cameraParam is None:
			messagebox.showinfo(
				title='Warning', message='Please do camera calibration first')
			return
		xodr_filename = self.videoFilename+'_RoadNet.xodr'
		bound_pixel = generate_road(self.cameraParam,
									self.roadBoundPts, self.laneSettingFrame.laneNum,
									(self.imgRaw.shape[1], self.imgRaw.shape[0]), 
									self.imgScale, self.imgPadding,
									xodr_filename,self.RoadDataList_N,self.RoadDataList_W,self.RoadDataList_S,self.RoadDataList_E)
		saver_filename = self.videoFilename+'_road_param.pkl'
		try: 
			with open(saver_filename, 'wb') as f:
				pickle.dump([self.roadFrameID, bound_pixel,
							self.laneSettingFrame.laneNum], f)
		except Exception as e: 
			print('Failed to write lane results.')
	def load_road_results(self): 
		saver_filename = self.videoFilename+'_road_param.pkl'
		if os.path.exists(saver_filename): 
			raw = pickle.load(open(saver_filename, 'rb'))
			self.roadFrameID = raw[0]
			bound_pixel = raw[1]
			self.roadLaneNum = raw[2]
			for o in ORIENTATION_LIST:
				self.roadBoundPts[o] = convert_pixel_to_canvas([bound_pixel[o]], self.imgScale, self.imgPadding)[0]


	def start_sim_lib(self):
		os.system('python3 SimConfig_v2.py')
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
		if c == "'\\x7f'":	# delete press
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
		if c == "'\\x7f'":	# delete press
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
			#self.calib_disp_results(canvas_coord=[event.x, event.y])
			self.show_zoomed_focus(canvas_coord=[event.x, event.y])

	#mouse move in the perspective view
	def mouse_moveP(self, event): 
		if not self.dragRectObjs is None: 
			for i in range(len(self.dragRectObjs)):	 
				self.dragRectObjs[i].highlight(False)
		if SHOW_ZOOMED_FOCUS: 
			self.show_zoomed_perspective(canvas_coord=[event.x, event.y])
	
	def mouse_click(self, event):
		if self.imgRaw is None: 
			return
		if self.currHitTab =='Detect&Track':# only work in detection tab
			if self.InsertObjectMode: #only work in insert	mode
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
		elif self.currHitTab =='Calibration': 
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
			if self.calAddPLMode and bool(self.tempInsertObjectData): #only work in insert	mode
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
		elif self.currHitTab=='Calibration': 
			if self.calAddPLMode and not self.calAddEditMode and bool(self.tempParallelData): #only work in insert	mode
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
			#self.calib_disp_results(canvas_coord=[event.x, event.y])
			self.show_zoomed_focus(canvas_coord=[event.x, event.y])

	def mouse_release(self, event):
		if self.imgRaw is None: 
			return
		need_update = False
		if self.currHitTab =='Calibration':	 # calibaration mode
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
					if len(self.tempPLLine) ==2: #collect 2 lines 
						temp_points = []
						for obj in self.tempPLLine: 
							oid = obj.objectID
							pts = obj.points
							temp_points.append([oid, pts[0][0],pts[0][1],pts[1][0], pts[1][1]])
						self.calParaLines.append(temp_points)
						self.calParaLinesObjIds.append(self.tempPLLine)
						self.tempPLLine = []
			elif not self.calAddPLMode and not self.calAddEditMode and bool(self.tempRefPointData): 
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
						self.calRefPoints.append([oid, points[0], points[1], points[2],points[3], w.value])
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
									ObjectID=self.nextObjectID)	 # canvasPaintFrame)
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
		'''	   
		pass #do nothing
		if need_update:
			self.update_message_boxes()	 # update result message box
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
		self.load_image_data(self.currFrameId)
		self.clean_draw_objects()
		self.draw_image(self.imgDisp)
		self.draw_annotations()
		self.update_message_boxes()
		self.statusFrameLabel.config(text ='Frame {}/{}'.format(self.currFrameId+1, self.totalFrameNum))
		self.frameLabel.config(text ='Frame {}'.format(self.currFrameId+1))
	
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

	def generate_thumbnail_view(self): 
		self.frameTreeView['columns']=("frame", "anno")
		self.frameTreeView.column("#0", width=self.icon_image_size[0]*3, stretch=NO)
		col_width = (self.frameTreeView.winfo_width()-self.icon_image_size[0]*3) // 2 
		self.frameTreeView.column("frame", width=col_width,anchor='w')
		self.frameTreeView.column("anno", width=col_width)
		self.frameTreeView.heading("frame", text="Frame")
		self.frameTreeView.heading("anno", text="Annotation")
		self.frameTreeView.delete(*self.frameTreeView.get_children())#clean first

		for i in range(self.totalFrameNum): 
			self.frameTreeView.insert('', 'end',# text ='', 
							values=('Frame {}'.format(i+1),''),
							tags=str(i), 
							image =self.empty_tkimg)
	def treeview_click(self, event): 
		item = self.frameTreeView.identify("item", event.x, event.y)
		text = self.frameTreeView.item(item)['tags']
		try: 
			fid = int(text[0])
			self.currFrameId = fid
			self.goto()
		except: 
			return
		

if __name__ == '__main__':
	root = Tk()
	
	# root.geometry('800x600')
	imgicon = PhotoImage(file='./icon/icon.gif')
	root.tk.call('wm', 'iconphoto', root._w, imgicon)
	#windowWidth = 1200	 # root.winfo_reqwidth()
	#windowHeight = 760	 # root.winfo_reqheight()
	windowWidth = 1920	# root.winfo_reqwidth()
	windowHeight = 1080	 # root.winfo_reqheight()
	positionRight = int(root.winfo_screenwidth()/2 - windowWidth/2)
	positionDown = int(root.winfo_screenheight()/2 - windowHeight/2)
	root.geometry("{}x{}+{}+{}".format(windowWidth,
									   windowHeight, positionRight, positionDown))
	tool = MainGUI(root)
	#print(root.cget('bg'))
	root.mainloop()
