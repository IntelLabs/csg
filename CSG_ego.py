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
from src.ui.lane_edit_frame import LaneEditFrame
from src.ui.canvas import ZoomFrame	 # , ZoomFrame2
from src.ui.image_utils import *
from src.ui.drag_line import DragLine
from src.calibration.calib_main import camera_clibration_canvas
from src.detect.detect_factory import Detecter
from src.track.track_factory import	 TRACKER
from src.track.trajectory import postprocessing_trajectory,select_trajectory, auto_refine
from src.road.road_main import generate_egoview_straight_road
from src.error_check import multi_frame_error_check
from config import DET_CONFIG,TRACK_CONFIG, SOURCE, LANE_CONFIG
from src.road.lane import LaneDetect#, LaneCluster
from src.convert_util import project_2d_to_3d_depth,get_abs_slam_pose
from src.ui.lane_edit_frame import View_AskLaneID

'''
Note: 
All coordinates loaded in variables are canvas coordiantes(for drawing)
Saved coordinates are pixel-based, so need conversion when save/load

'''
WINDOW_WIDTH =1200 #1920 
WINDOW_HEIGHT=760 #1080
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 480
#CANVAS_WIDTH = 1400
#CANVAS_HEIGHT = 800
ORIENTATION_LIST =	['North', 'West', 'South', 'East']
RADIUS = 7#to draw oval in road
LINEWIDTH = 2 #draw line
SMALL_CANVAS_SIZE=200

SHOW_ZOOMED_FOCUS = True

class MainGUI:
	def __init__(self, master):
		self.parent = master
		self.parent.title("CSG ToolKit v1.0(Ego)")

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
		self.titleLabel.pack(fill=X, side=LEFT, expand=1)
		self.openBtn.pack(fill=X, side=RIGHT, padx=20, expand=0)

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
		self.tabControl.pack(expand=1, fill="both")	 # Pack to make visible

		# tab-calibration
		self.calibrationFrame = Frame(self.tabCalib)
		self.calibrationFrame.pack(fill=X, pady=5, side=TOP)
		self.calSlamBtn = Button(self.calibrationFrame, text="Auto Calibration", command=self.auto_calib)
		self.calSlamBtn.pack(fill=X, pady=1, side=TOP)
		#self.calDepthBtn = Button(self.calibrationFrame, text="Estimate Depth", command=self.estimate_depth)
		#self.calDepthBtn.pack(fill=X, pady=1, side=TOP)		
		self.calScaleBtn = Button(self.calibrationFrame, text="Scale Refine", command=self.scale_refine)
		self.calScaleBtn.pack(fill=X, pady=1, side=TOP)
		ttk.Separator(self.calibrationFrame, orient=HORIZONTAL).pack(fill=BOTH, pady=10, side=TOP)
		Label(self.calibrationFrame, text="Camera Intrinsic parameter").pack(fill=X, side=TOP)
		self.calIntrinFrame = Frame(self.tabCalib)
		self.calIntrinFrame.pack(fill=X, side=TOP)
		Label(self.calIntrinFrame, text="Focal length").grid(row=0, column=0, columnspan=4, sticky='we')
		Label(self.calIntrinFrame, text="x").grid(row=1, column=0, columnspan=1, sticky='we')
		vcmd = (master.register(self.validateFloat), '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
		self.fxEntry = Entry(self.calIntrinFrame, validate = 'key', validatecommand = vcmd)
		self.fxEntry.grid(row=1, column=1, sticky='we')
		Label(self.calIntrinFrame, text="y").grid(row=1, column=2, columnspan=1, sticky='we')
		self.fyEntry = Entry(self.calIntrinFrame,validate = 'key', validatecommand = vcmd)
		self.fyEntry.grid(row=1, column=3, sticky='we')
		Label(self.calIntrinFrame, text="Principal point").grid(row=2, column=0, columnspan=4, sticky='we')
		Label(self.calIntrinFrame, text="x").grid(row=3, column=0, columnspan=1, sticky='we')
		self.cxEntry = Entry(self.calIntrinFrame,validate = 'key', validatecommand = vcmd)
		self.cxEntry.grid(row=3, column=1, sticky='we')
		Label(self.calIntrinFrame, text="y").grid(row=3, column=2, columnspan=1, sticky='we')
		self.cyEntry = Entry(self.calIntrinFrame,validate = 'key', validatecommand = vcmd)
		self.cyEntry.grid(row=3, column=3, sticky='we')
		
		#ttk.Separator(self.calibrationFrame, orient=HORIZONTAL).pack(fill=BOTH, pady=20, side=TOP)
		
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
		self.roadLaneDetBtn = Button(self.tabRoad, text="Auto detect lane", command=self.lane_detect)
		self.roadLaneDetBtn.pack(fill=X, pady=10, side=TOP)
		self.roadLaneEditBtn = Button(self.tabRoad, text="Edit lane", command=self.lane_edit)
		self.roadLaneEditBtn.pack(fill=X, pady=2, side=TOP)
		self.roadCenterLaneSetBtn = Button(self.tabRoad, text="Set reference lane", command=self.set_center_lane)
		self.roadCenterLaneSetBtn.pack(fill=X, pady=2, side=TOP)
		self.roadGenBtn = Button(self.tabRoad, text="Generate Road", command=self.generate_road_net)  # self.GenerateRoadNet
		self.roadGenBtn.pack(fill=X, pady=2, side=TOP)


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
		self.imgRaw = None	# cv mode(bgr)
		self.imgDisp = None
		self.imgScale = None
		self.imgPadding = None
		self.imgRoad = None
		self.imgRoadDisp = None
		self.tkimg = None
		self.detector = None
		self.tracker = None
		self.ObjectList = None
		self.ObjectList_pixel = None
		self.LaneList = None
		self.LaneList_pixel = None
		
		#TODO: auto_detect and manually set
		self.intrinsic = None#np.array([[721.5377, 0, 609.5593],[0, 721.5377,172.854 ],[0, 0,       1]]) 
		self.raw_pose = None
		self.abs_pose = None
		self.pose_scale = None
		self.raw_depth=None
		self.abs_depth = None
		self.depth_scale = 0
		self.center_lane_ID = -1

		self.lane2d = dict()
		self.trajectory = None
		self.nextObjectID = 1
		self.imgPerspective = None	# cv mode(bgr)

		self.cameraParam = None
		self.dragRectObjs = None
		self.labelList =['Car', 'Bus','Pedestrian','Rider', 
						'Truck', 'Motorcycle','Bicycle','Tricycle']#TODO: read/write to file 
		
		# mode-related variables
		self.currHitTab = ""  # text
		self.playMode = True #play or pause
		self.InsertObjectMode = False #add or edit objects in canvas

		# temporay variables used to recording intermediate status
		self.tempDragData = None  # drag
		self.tempInsertObjectData = dict()
			
		#icon image
		self.icon_image_size=(20,20)
		warning_icon = Image.open('icon/warning.png')
		warning_thumb=ImageOps.fit(warning_icon, self.icon_image_size, Image.ANTIALIAS)
		self.warning_tkimg = ImageTk.PhotoImage(warning_thumb)
		pass_icon = Image.open('icon/pass.png')
		pass_thumb=ImageOps.fit(pass_icon, self.icon_image_size, Image.ANTIALIAS)
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
			for k, anno in enumerate(curr_anno): 
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
				pix_coord = self.ObjectList_pixel[self.currFrameId][k]
				pt1 = convert_pixel_to_canvas([[pix_coord[2], pix_coord[3]]], self.imgScale, self.imgPadding)
				pt2 = convert_pixel_to_canvas([[pix_coord[4], pix_coord[5]]], self.imgScale, self.imgPadding)
				points = [pt1[0], pt2[0]]
				#points =[[anno[2], anno[3]],[anno[4], anno[5]]]
				
				ls = DragRect(self, self.canvasPaintFrame.canvas,points, color, 
							 title = textstr, objectID=id)
				self.dragRectObjs.append(ls)
			self.show_trajectory(False)
			self.show_trajectory(True)
		elif self.currHitTab=='Calibration':
			pass
		elif self.currHitTab=='Road Reconstruction':
			#draw lane 
			if not bool(self.lane2d): 
				return 
			if self.currFrameId not in self.lane2d.keys(): 
				return 
			lane2d_pixel = self.lane2d[self.currFrameId]
			#convert pixel to canvas 
			text_font = ('Helvetica', 14) 
			for _, (lane_id,lane_pixels) in enumerate(lane2d_pixel.items()):
				canvas_coords = [] 
				for pt in lane_pixels: 
					c = convert_pixel_to_canvas([pt], self.imgScale, self.imgPadding)[0]
					if lane_id == self.center_lane_ID: 
						fill_color = 'red'
					else: 
						fill_color = 'orange'
					self.canvasPaintFrame.canvas.create_oval(c[0] - 5, c[1] - 5,
															 c[0] + 5, c[1] + 5,
															 fill=fill_color, outline= 'black', tags='lane_point')
					canvas_coords.append(c)	
				for j in range(len(canvas_coords)-1): 
					pt1 = canvas_coords[j]
					pt2 = canvas_coords[j+1]
					if lane_id == self.center_lane_ID: 
						fill_color = 'yellow'
					else: 
						fill_color = 'olive'			
					self.canvasPaintFrame.canvas.create_line(pt1[0], pt1[1],
															pt2[0],pt2[1],
															width=4, fill=fill_color, 
															tags='lane_line')										
				if lane_id == self.center_lane_ID:
					fill_color = 'cyan'
				else: 
					fill_color = 'blue'
				self.canvasPaintFrame.canvas.create_text(pt1[0]+15, pt1[1], 
                                            text=str(lane_id), fill='green', font=text_font,tags='lane_text')
		
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
			pass
				
		elif self.currHitTab=='Road Reconstruction':
			pass #no message box in this tab

	def clean_draw_objects(self):
		#  clean all annotations, keep image
		self.canvasPaintFrame.canvas.delete('dragRect')
		self.canvasPaintFrame.canvas.delete('parallel_line')
		self.canvasPaintFrame.canvas.delete('reference_line')
	   # self.canvasPaintFrame.canvas.delete('line')
		self.canvasPaintFrame.canvas.delete('lane_point')
		self.canvasPaintFrame.canvas.delete('lane_line')
		self.canvasPaintFrame.canvas.delete('lane_text')
		self.show_trajectory(False)

	def reset_modes(self, clean_results=False):
		c = self.parent.cget('bg')
		
		self.pauseMode = False 
		if self.ObjectList is None and self.totalFrameNum>0: #if not initialize
			self.ObjectList = [[] for _ in range(self.totalFrameNum)]
			self.ObjectList_pixel = [[] for _ in range(self.totalFrameNum)]
		self.InsertObjectMode = False
		self.insertObjectBtn['state'] = 'normal'
		self.modifyObjectBtn['state'] = 'disable'
		#default button color 
		self.modifyObjectBtn.configure(bg='#6495ed')
		self.insertObjectBtn.configure(bg=c)

		if clean_results: 
			self.ObjectList = None
			self.ObjectList_pixel = None
			self.cameraParam = None
			self.raw_depth = None 
			self.abs_depth = None
			self.depth_scale = 0
			self.raw_pose = None 
			self.abs_pose = None
			self.pose_scale  = None
			self.intrinsic = None
			self.lane2d=dict()
			self.trajectory = None
			


	# --------tab1:-Calibration-----------
	
		
			

	#Display the pespective view for fine tuning
	def calib_disp_results(self, canvas_coord=None):
		if self.imgRaw is None: 
			return
		self.animationDisplayCanvas.delete('all')
		#TODO

		if self.cameraParam is None:
			return 
		#fpitch = self.calPitchScale.get()
		#froll = self.calRollScale.get()
		#fyaw = self.calYawScale.get()

		#note the perspective image is a square image, different resolution with raw image
		print("start calc perspective view")
		self.imgPerspective, self.cameraParam = convert_perspective_image(self.imgRaw, 
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
		view_name = 'Ego-view'
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
			#track return: (trk_id, object_label_id, det_box_id, x1,y1,x2,y2)
			curr_det_rst,_ =self.detector.detect(self.imgRaw)
			
			self.tracker.update(curr_det_rst, self.imgRaw, frame_id)
			curr_tracklets = self.tracker.get_current_result()
			
			curr_objs = []
			curr_objs_pixel = []
			for tr in curr_tracklets: 
				lt = convert_pixel_to_canvas([[tr[3], tr[4]]], self.imgScale, self.imgPadding)[0]
				rb =  convert_pixel_to_canvas([[tr[5], tr[6]]], self.imgScale, self.imgPadding)[0] 
				source = SOURCE['Auto'] #from tracking 
				curr_objs.append([tr[0], tr[1], lt[0], lt[1], rb[0], rb[1], tr[2], source])
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
		#TODO: do not save canvas-coordinate annotations
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
		from src.file_parser import export_traj_xml_file_ego
		tags = os.path.splitext(self.videoFilename)
		traj_xml_file = tags[0]+'.xml'
		fps = self.videoCap.get(5)#cv2.CAP_PROP_FPS)

		export_traj_xml_file_ego(traj_pixel, self.intrinsic,self.abs_pose,self.abs_depth,
						         traj_xml_file, self.videoFilename[:-4], fps, auto_fill=0)
 

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

	def load_road_results(self): 
		saver = self.videoFilename+"_lane.pkl"
		if os.path.exists(saver): 
			raw = pickle.load(open(saver, 'rb'))
			self.lane2d = raw[0]
			self.center_lane_ID = raw[1]
	
	def save_road_results(self): 
		if bool(self.lane2d):
			saver = self.videoFilename+"_lane.pkl"
			try: 
				with open(saver, 'wb') as f: 
					pickle.dump([self.lane2d,self.center_lane_ID], f)
			except Exception as e: 
				print('Failed to write lane results')
	


	# --------Mouse event capture -----------


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
			pass 
				
		

	def mouse_drag(self, event):
		if self.currHitTab =='Detect&Track':# only work in detection tab
			if self.InsertObjectMode and bool(self.tempInsertObjectData): #only work in insert	mode
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
			pass
		if SHOW_ZOOMED_FOCUS: 
			#self.calib_disp_results(canvas_coord=[event.x, event.y])
			self.show_zoomed_focus(canvas_coord=[event.x, event.y])

	def mouse_release(self, event):
		if self.imgRaw is None: 
			return
		need_update = False
		if self.currHitTab =='Calibration':	 # calibaration mode
			pass	   

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
   
			pass #do nothing
		if need_update:
			self.update_message_boxes()	 # update result message box
		return

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
	def validateFloat(self, action, index, value_if_allowed,prior_value, text, validation_type, trigger_type, widget_name):
		if value_if_allowed:
			try:
				float(value_if_allowed)
				return True
			except ValueError:
				return False
		else:
			return False
	
	def auto_calib(self): 
		#orbslam+monodepth
		from src.file_parser import parse_slam_file, interp_slam_keyframe, read_kittti_calib_file
		from config import DEPTH_CONFIG
		popup_msg =Toplevel()#,takefocus=True)
		x = self.parent.winfo_x()
		y = self.parent.winfo_y()
		popup_msg.geometry("%dx%d+%d+%d" % (400,80, x + 400, y + 200))
		popup_msg.grab_set()
		popup_msg.resizable(False, False)
		popup_msg_lbl = Label(popup_msg, text="Depth Estimation")
		popup_msg_lbl.pack()#grid(row=0,column=0)
		progress_var = DoubleVar()
		progress_bar = ttk.Progressbar(popup_msg, variable=progress_var, maximum=100,
                         orient=HORIZONTAL, length=350)
		progress_bar.pack(side=TOP)
		popup_msg.pack_slaves()
		progress_var.set(0)
		popup_msg.update()

		from src.depth.monodepth import DepthEstimator
		depth_model = DepthEstimator()
		depth_model.build_model(DEPTH_CONFIG['MODEL'])
		self.raw_depth= []
		for frame_id in range(self.totalFrameNum): 
			progress_var.set(round((frame_id/(self.totalFrameNum+50))*100))
			popup_msg_lbl.config(text='Depth Estimation: {}/{}'.format(frame_id+1, self.totalFrameNum))
			popup_msg.update()
			
			self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
			_, bgr = self.videoCap.read()
			curr_depth =  depth_model.estimate_image(bgr)
			self.raw_depth.append(curr_depth)
		
		self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, self.currFrameId)
		depth_model.release()
		progress_var.set(80)
		popup_msg_lbl.config(text='Waiting!')
		popup_msg.update()

		slam_rst_file = os.path.join(os.path.dirname(self.videoFilename),
								 'KeyFrameTrajectory.txt')
		slam_time_file = os.path.join(os.path.dirname(self.videoFilename),
									'times.txt')
		slam_calib_file = os.path.join(os.path.dirname(self.videoFilename),
								 'calib.txt')
		if os.path.exists(slam_rst_file) and os.path.exists(slam_time_file) and os.path.exists(slam_calib_file):
			self.intrinsic = read_kittti_calib_file(slam_calib_file)
			slam_valid_frames, slam_poses_mat = parse_slam_file(slam_rst_file, slam_time_file)
			self.raw_pose, _ = interp_slam_keyframe(slam_poses_mat, slam_valid_frames)
			self.pose_scale = np.array([8.09901665, 29.54727539, 27.75993509])#11.78, 12.44, 12.05])
			self.depth_scale = 15.67#33.45
		else:
			fx_str = self.fxEntry.get()
			fy_str = self.fyEntry.get()
			cx_str = self.cxEntry.get()
			cy_str = self.cyEntry.get()
			if fx_str != '' and fy_str != '' and cx_str != '' and cy_str != '':
				from src.file_parser import write_kittti_calib_file
				### create calib file
				write_kittti_calib_file(os.path.join(os.path.dirname(self.videoFilename),'calib.txt'),[float(fx_str), float(fy_str), float(cx_str), float(cy_str)])
					### create times file
				videoCap = cv2.VideoCapture(self.videoFilename)
				frame_rate = videoCap.get(5)
				frame_count = videoCap.get(7)
				print(frame_count)
				time_list = []
				with open(os.path.join(os.path.dirname(self.videoFilename),'times.txt'), 'w') as f:
					for i in range((int(frame_count))):
						time_temp="%.6f" % (i*1/frame_rate)
						time_list.append(time_temp)
						f.write(time_temp+' \n')
					### prepare yaml file for SLAM ###
				image_folder= os.path.join(os.path.dirname(self.videoFilename))
				success, image = videoCap.read()
				count = 0
				success = True
				if not os.path.exists(image_folder+'/image_0'):
					os.mkdir(image_folder+'/image_0')
				while success:
					cv2.imwrite(image_folder+'/image_0/%06d.png' % count, image)
					success, image = videoCap.read()
					count += 1

				with open('./src/SLAM/SLAM.yaml', 'r') as f:
					list_read = f.readlines()

				list_read[5] = 'Camera.fx: ' + fx_str+'\n'
				list_read[6] = 'Camera.fy: ' + fy_str+'\n'
				list_read[7] = 'Camera.cx: ' + cx_str+'\n'
				list_read[8] = 'Camera.cy: ' + cy_str+'\n'
				with open('./src/SLAM/SLAM.yaml', 'w') as f:
					for i in range(len(list_read)):
						f.write(list_read[i])
				f.close()
				###start slam ###
				os.system('./src/SLAM/mono_kitti  ./src/SLAM/ORBvoc.txt ./src/SLAM/SLAM.yaml' + ' ' + image_folder)

				### get traj file and remove pics ###
				import shutil
				shutil.move('KeyFrameTrajectory.txt',image_folder+'/KeyFrameTrajectory.txt')
				#os.rmdir(image_folder+'/image_0')

				slam_rst_file = os.path.join(os.path.dirname(self.videoFilename),
										 'KeyFrameTrajectory.txt')
				slam_time_file = os.path.join(os.path.dirname(self.videoFilename),
											'times.txt')
				slam_calib_file = os.path.join(os.path.dirname(self.videoFilename),
										 'calib.txt')

				self.intrinsic = read_kittti_calib_file(slam_calib_file)
				slam_valid_frames, slam_poses_mat = parse_slam_file(slam_rst_file, slam_time_file)


				self.raw_pose, _ = interp_slam_keyframe(slam_poses_mat, slam_valid_frames)
				self.pose_scale = np.array([8.09901665, 29.54727539, 27.75993509])#11.78,12.44,12.05])
				self.depth_scale = 15.67#33.45
			else:
				messagebox.showinfo(
						title='Warning', message='Please Setting camera instrinct parameters and autocalib again')
				return
			
		from src.scale.auto_scale_solver import ScaleRecovery
		height,width = self.raw_depth[0].shape[0], self.raw_depth[0].shape[1]
		scalesolver = ScaleRecovery(1, height, width).cuda()	
		
		self.depth_scale, self.abs_depth = scalesolver.get_depth_scale(self.raw_depth, self.intrinsic, real_cam_height=1.65)
		#abs_depth = []
		#for d in self.raw_depth: 
		#	abs_depth.append(d*self.depth_scale)
		#self.abs_depth = abs_depth
		#self.pose_scale = scalesolver.get_translation_scale(self.videoCap, self.intrinsic, 
		#					self.abs_depth, self.raw_pose, self.ObjectList_pixel)
		self.abs_pose = get_abs_slam_pose(self.raw_pose, self.pose_scale)

		popup_msg.destroy()

		self.save_calib_results()
		fx,fy = self.intrinsic[0,0],self.intrinsic[1,1]
		cx,cy = self.intrinsic[0,2], self.intrinsic[1,2]
		self.fxEntry.delete(0, END)
		self.fxEntry.insert(0, "%.2f" % fx)
		self.fyEntry.delete(0, END)
		self.fyEntry.insert(0, "%.2f" % fy)
		self.cxEntry.delete(0, END)
		self.cxEntry.insert(0, "%.2f" % cx)
		self.cyEntry.delete(0, END)
		self.cyEntry.insert(0, "%.2f" % cy)
		
	
	def scale_refine(self): 
		os.system('python3 ScaleSolver.py --videoname '+self.videoFilename)
		depth = []

		file_path = os.path.join(os.path.dirname(self.videoFilename), 'ScaleSolver_result.txt')

		with open(file_path, 'r') as f:
			raw_lines = f.readlines()
		p0 = raw_lines[0].split(' ')[-1]
		p1 = raw_lines[1].split(' ')[-1]
		p2 = raw_lines[2].split(' ')[-1]
		tx_scale = float(p0)
		tz_scale = float(p1)
		ty_scale = float(p2)
		self.pose_scale = np.array([tx_scale,ty_scale,tz_scale])

		for d in self.raw_depth: 
			d = d*self.depth_scale 
			depth.append(d)
		self.abs_depth = depth
		self.abs_scale = get_abs_slam_pose(self.raw_pose, self.pose_scale)
		self.save_calib_results()
			
		
	def lane_detect(self): 
		if self.imgRaw is None: 
			return
		if (self.intrinsic is None) or (self.abs_pose is None) or (self.abs_depth is None): 
			print('please run clibration first')
			return 
		
		if not self.LaneList is None: 
			msgbox = messagebox.askquestion("Notice", "AutoRun with clean all current lanes. Are you sure you want to continue?",icon='warning')
			if not msgbox =='yes': 
				return 
		lane_detector = LaneDetect(LANE_CONFIG['MODEL'], device=LANE_CONFIG['DEVICE']) #bug when use cpu
		#detect frame-by-frame 
		old_frame_id = self.currFrameId
		det_lanes = []
		for frame_id in range(self.totalFrameNum): 
			self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
			_, bgr = self.videoCap.read()		
			lx,ly,_ = lane_detector.detect_image(bgr) #lane_im for viz
			lanes = []
			num_lanes = len(lx[0])
			for j in range(num_lanes): 
				lanes.append(np.array(list(zip(lx[0][j], ly[0][j]))))
			det_lanes.append(lanes)

		self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, old_frame_id)
		lane_detector.release_from_gpu()
		del lane_detector

		#cluster and refine in 3d&2d 
		from src.road.lane import lane_3d_reconstruct
		self.lane2d = lane_3d_reconstruct(det_lanes,
										  self.intrinsic,
										  self.abs_pose,
										  self.abs_depth) 
	
		self.save_road_results()
		self.clean_draw_objects()
		self.draw_annotations()
	def lane_edit(self): 
		#call lane_edition ui. 
		if self.videoCap is None: #need to open video first
			return
		
		param = dict()
		param['intrisic'] = self.intrinsic
		param['extrisic'] = self.abs_pose
		param['depth']  = self.abs_depth
		w = LaneEditFrame(self.parent, self.videoCap, self.lane2d, param)
		self.parent.wait_window(w.top)
		self.lane2d = w.lane_2d
		self.save_road_results()
		self.clean_draw_objects()
		self.draw_annotations()
	def load_calib_results(self):
		#TODO, load slam result
		saver = self.videoFilename+"_calib.pkl"
		if os.path.exists(saver):
			raw  = pickle.load(open(saver, 'rb'))
			self.raw_depth = raw[0]
			self.raw_pose  = raw[1]
			self.intrinsic = raw[2]
			self.depth_scale = raw[3]
			self.pose_scale = raw[4]
			self.abs_depth = raw[5] 
			self.abs_pose = raw[6]
			fx,fy = self.intrinsic[0,0],self.intrinsic[1,1]
			cx,cy = self.intrinsic[0,2], self.intrinsic[1,2]
			self.fxEntry.delete(0, END)
			self.fxEntry.insert(0, "%.2f" % fx)
			self.fyEntry.delete(0, END)
			self.fyEntry.insert(0, "%.2f" % fy)
			self.cxEntry.delete(0, END)
			self.cxEntry.insert(0, "%.2f" % cx)
			self.cyEntry.delete(0, END)
			self.cyEntry.insert(0, "%.2f" % cy)
		else: 
			self.raw_depth, self.raw_pose, self.intrinsic = None, None, None
			self.abs_depth, self.abs_pose = None, None
			self.depth_scale, self.pose_scale = 0.0, None

	def save_calib_results(self): 
		saver = self.videoFilename +"_calib.pkl"
		try: 
			with open(saver, 'wb') as f: 
				pickle.dump([self.raw_depth ,self.raw_pose, self.intrinsic, 
							self.depth_scale, self.pose_scale, 
							self.abs_depth, self.abs_pose], f)
		except Exception as e: 
			print('Failed to write calibration results.')
			
	def set_center_lane(self): 
		w = View_AskLaneID(self.parent)
		self.parent.wait_window(w.top)
		if int(w.value)>=0:
			self.center_lane_ID = int(w.value)
		self.save_road_results()
		self.clean_draw_objects()
		self.draw_annotations()
		

	def generate_road_net(self): 
		#TODO, generate_road_ego

		if self.center_lane_ID <0: 
			print('please select center lane first!')
			return
		tags = os.path.splitext(self.videoFilename)
		xodr_filename = tags[0]+'_map.xodr'
		generate_egoview_straight_road(self.lane2d, self.center_lane_ID, 
					self.intrinsic, self.abs_pose, self.abs_depth,
					xodr_filename)

if __name__ == '__main__':
	root = Tk()
	
	# root.geometry('800x600')
	imgicon = PhotoImage(file='./icon/icon.gif')
	root.tk.call('wm', 'iconphoto', root._w, imgicon)
	#windowWidth = 1920	# root.winfo_reqwidth()
	#windowHeight = 1080	 # root.winfo_reqheight()
	positionRight = int(root.winfo_screenwidth()/2 - WINDOW_WIDTH/2)
	positionDown = int(root.winfo_screenheight()/2 - WINDOW_HEIGHT/2)
	root.geometry("{}x{}+{}+{}".format(WINDOW_WIDTH,
									   WINDOW_HEIGHT, positionRight, positionDown))
	tool = MainGUI(root)
	#print(root.cget('bg'))
	root.mainloop()
