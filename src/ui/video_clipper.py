
from tkinter import *
from tkinter import ttk, messagebox,filedialog
import cv2
from PIL import Image as PILImage
from PIL import ImageTk
from .image_utils import fit_image_to_canvas
#import image_utils as imgUtil
#compatible to all resolutions, both horizontal and vertical

class VideoClipperWindow(Toplevel): 
    def __init__(self, master, videoCap, width=600, height=500): 
        top = self.top = self
        self.width= width
        self.height = height
        self.videoCap = videoCap
        Toplevel.__init__(self, master, width=self.width, height = self.height)
        x = master.winfo_x()
        y = master.winfo_y()
        self.geometry('{}x{}+{}+{}'.format(width,height, x+200,y+50))
        self.protocol("WM_DELETE_WINDOW",self.destroy)
        #self.attributes('-topmost',True)
        self.transient()
        self.resizable(False, False)
        self.title = Label(self, text="Frame")
        self.title.pack(side=TOP)
        self.title.config(font=("Courier", 20))

        self.canvas_height = self.height-200
        self.canvas = Canvas(self, highlightthickness=0,
                            width = self.width, height = self.canvas_height,
                            background="#90AFC5")
        self.canvas.pack(side=TOP)
        self.navigation_frame = Frame(self)
        self.navigation_frame.pack(side=TOP, ipady=20)
        self.set_frame=Frame(self)
        self.set_frame.pack(side=TOP, ipady=20)
        self.command_frame = Frame(self)
        self.command_frame.pack(side=TOP, ipady=20)

        self.gotoFirstBtn = Button(self.navigation_frame, text="|<", command=self.goto_first)
        self.gotoPrev10Btn = Button(self.navigation_frame, text="<<", command=self.goto_prev10)
        self.gotoPrevBtn = Button(self.navigation_frame, text="<", command=self.goto_prev)
        self.playBtn = Button(self.navigation_frame,text="|>", command=self.play)
        self.gotoNextBtn = Button(self.navigation_frame, text=">", command=self.goto_next)
        self.gotoNext10Btn = Button(self.navigation_frame, text=">>", command=self.goto_next10)
        self.gotoLastBtn = Button(self.navigation_frame, text=">|", command=self.goto_last)
        self.gotoFirstBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
        self.gotoPrev10Btn.pack(fill=X, side=LEFT, padx=1, expand=0)
        self.gotoPrevBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
        self.playBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
        self.gotoNextBtn.pack(fill=X, side=LEFT, padx=1, expand=0)
        self.gotoNext10Btn.pack(fill=X, side=LEFT, padx=1, expand=0)
        self.gotoLastBtn.pack(fill=X, side=LEFT, padx=1, expand=0)

        self.startVar = StringVar()
        self.endVar = StringVar()
        self.startVar.trace("w", lambda name, index, mode, sv=self.startVar: self.start_callback(sv))
        self.endVar.trace("w", lambda name, index, mode, sv=self.endVar: self.end_callback(sv))
        
        self.setStartBtn = Button(self.set_frame, text="Set as Start", command=self.set_start)
        self.setStartBtn.pack(fill=X, side=LEFT, padx=5, expand=0)
        self.startEntry = Entry(self.set_frame, textvariable=self.startVar)
        self.startEntry.pack(fill=X, side=LEFT, padx=5, expand=0)
        Label(self.set_frame, text="-").pack(fill=X, side=LEFT, padx=1, expand=0)
        self.endEntry = Entry(self.set_frame,textvariable=self.endVar)
        self.endEntry.pack(fill=X, side=LEFT, padx=5, expand=0)
        self.setEndBtn = Button(self.set_frame, text="Set as End", command=self.set_end)
        self.setEndBtn.pack(fill=X, side=LEFT, padx=5, expand=0)


        self.clipBtn = Button(self.command_frame, text="Clip&Save", command=self.clip_save)
        self.cancelBtn = Button(self.command_frame, text="Cancel", command=self.cancel)

        self.clipBtn.pack(side=LEFT, padx=10, expand=False)
        self.cancelBtn.pack(side=RIGHT, padx=10, expand=False)

        self.currFrameId  = 0
        self.pauseMode = False
        self.totalFrameNum = int(self.videoCap.get(7))
        self.saver_filename = None

        self.goto()
    def start_callback(self, sv): 
        char = sv.get()
        try: 
            val = int(char)
        except: 
            sv.set("")
            return 
        if val <0 or val>self.totalFrameNum-2:
            sv.set("") 

    def end_callback(self, sv): 
        char = sv.get()
        try: 
            val = int(char)
        except: 
            sv.set("")
            return
        if val <1 or val>self.totalFrameNum:
            sv.set("")  

    def set_start(self): 
        self.startEntry.delete(0,END)
        self.startEntry.insert(0,str(self.currFrameId+1))
        self.startVar.set(str(self.currFrameId+1))
       
    def set_end(self): 
        self.endEntry.delete(0,END)
        self.endEntry.insert(0,str(self.currFrameId+1))
        self.endVar.set(str(self.currFrameId+1))

    def goto(self): 
        textstr = "Frame: {}".format(self.currFrameId+1)
        self.title.config(text=textstr)
        self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, self.currFrameId)
        _, image_raw = self.videoCap.read()
        self.imgDisp, _, self.imgScale, self.imgPadding = fit_image_to_canvas(image_raw,
                                        desired_width=self.width, desired_height=self.canvas_height)
        rgb = cv2.cvtColor(self.imgDisp, cv2.COLOR_BGR2RGB)
        self.tkImg = ImageTk.PhotoImage(PILImage.fromarray(rgb))
        imageid = self.canvas.create_image(0,0,anchor='nw', image=self.tkImg)
        self.canvas.lower(imageid)  # set image into background
    
    def clip_save(self): 
        start_fid = int(self.startVar.get())-1
        end_fid = int(self.endVar.get())-1
        if start_fid <0 or end_fid<=start_fid:
            messagebox.showwarning("Warning", "Invalid start/end frame number!") 
            return
        saver_file = filedialog.asksaveasfilename(title='Save As',  filetypes=(("video files", "*.mp4"), ("video files", "*.avi")))
        if saver_file:
            self.saver_filename = saver_file
            width = int(self.videoCap.get(3))
            height = int(self.videoCap.get(4))
            fps = self.videoCap.get(5)
            suffix = saver_file[-4:]
            if suffix =='.avi': 
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
            elif suffix=='.mp4':
                fourcc =  cv2.VideoWriter_fourcc(*'mp4v')#MP4V')#(*'MPEG')#cv2.VideoWriter_fourcc('M','J','P','G')
            out = cv2.VideoWriter(saver_file,fourcc, fps, (width,height))
            if(out.isOpened()):
                for i in range(start_fid, end_fid+1): 
                    self.videoCap.set(cv2.CAP_PROP_POS_FRAMES, i)
                    _, image_raw = self.videoCap.read()
                    out.write(image_raw)
            out.release()                                                                
            self.destroy()
    def cancel(self): 
        self.saver_filename = None
        self.destroy()

    def play_video(self): 
        self.currFrameId += 1
        self.currFrameId = max(0, min(self.totalFrameNum-1, self.currFrameId))
        self.goto()
        if not self.pauseMode: 
            self.navigation_frame.after(15, self.play_video)
        if self.currFrameId == (self.totalFrameNum-1): #finished all play
            self.playBtn['text']='|>'
            self.pauseMode = True
    def play(self): 
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
        self.playBtn['text']='|>'
        self.pauseMode = True
        self.currFrameId = 0
        self.goto()

    def goto_prev(self):
        self.playBtn['text']='|>'
        self.pauseMode = True
        self.currFrameId = max(0, self.currFrameId-1)
        self.goto()

    def goto_prev10(self):
        self.playBtn['text']='|>'
        self.pauseMode = True
        self.currFrameId = max(0, self.currFrameId-10)
        self.goto()

    def goto_next(self):
        self.playBtn['text']='|>'
        self.pauseMode = True
        self.currFrameId = min(self.totalFrameNum-1, self.currFrameId+1)
        self.goto()

    def goto_next10(self):
        self.playBtn['text']='|>'
        self.pauseMode = True
        self.currFrameId = min(self.totalFrameNum-1, self.currFrameId+10)
        self.goto()

    def goto_last(self):
        self.playBtn['text']='|>'
        self.pauseMode = True
        self.currFrameId = self.totalFrameNum-1
        self.goto()
    
if __name__ == "__main__":
    root = Tk()
    videoFilename = filedialog.askopenfilename(title="Select Video", filetypes=(("video files", "*.mp4"), ("video files", "*.avi"),
                                                                                         ("all files", "*.*")))
    videoCap = cv2.VideoCapture(videoFilename)    
    v = VideoClipperWindow(root, videoCap)
    root.mainloop()