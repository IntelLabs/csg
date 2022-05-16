from tkinter import *
from tkinter import ttk, messagebox
#from src.ui.canvas import AutoScrollbar
from PIL import Image, ImageTk
#from src.ui.canvas import AutoScrollbar
import math
import queue
import threading
import os
import cv2
import numpy as np

class View_AskDistance(Toplevel):
    """
    A simple dialog that asks for a text value.
    """
    def __init__(self, master, value=u""):
        self.value = None
        Toplevel.__init__(self, master)
        self.protocol("WM_DELETE_WINDOW",self.destroy)
        self.attributes('-topmost',True)
        self.transient()
        self.resizable(False, False)
        top = self.top = self
        x = master.winfo_x()
        y = master.winfo_y()
        self.geometry("+%d+%d" % (x + 400, y + 200))
    #    top = self.top = tkinter.Toplevel(master)
    #    top.grab_set()
        self.l = ttk.Label(top, text=u"Distance Value(mm):")
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

        if value: self.e.insert(0, value)
        self.e.focus_set()
        top.bind('<Return>', self.save)

    def save(self, *_):
        self.value = self.e.get()
        try: 
            val = float(self.value)
        except: 
            messagebox.showinfo(title='Warning', message='Distance should be a number')
            self.e.focus_set()
            return 
        
        self.destroy()

    def cancel(self, *_): 
        self.value = 0
        self.destroy()

class View_AskIDLabel(Toplevel): 
    def __init__(self, master, labelList, ObjectID=0, label=None): 
        self.labelList = labelList
        self.ObjectID = ObjectID
        self.width = 300
        self.height = 500
        self.label = label
        Toplevel.__init__(self, master)
        self.protocol("WM_DELETE_WINDOW",self.destroy)
        self.attributes('-topmost',True)
        self.transient()
        self.resizable(False, False)
        x = master.winfo_x()
        y = master.winfo_y()
        self.geometry("%dx%d+%d+%d" % (self.width, self.height, x + 400, y + 200))
        top = self.top = self
        Label(top, text="Object ID").grid(row=0, column=0, sticky='w')
        self.id_entry= Entry(top)
        self.id_entry.grid(row=0, column=1, sticky='we')
        if self.ObjectID >0: #set default
            self.id_entry.insert(END, str(self.ObjectID))
        Label(top, text="Label").grid(row=1, column=0, sticky='we')
        self.label_entry = Entry(top)
        self.label_entry.grid(row=1, column=1, sticky='we')
        if self.label is not None: 
            self.label_entry.insert('0', self.label)
        self.label_entry.focus_set()
        self.labelListbox = Listbox(top, width=5, height=20)
        self.labelListbox.grid(row=2, column=0, columnspan=2, sticky='we')
        for item in self.labelList: 
            self.labelListbox.insert('end', item)
        Button(top, text="OK", command=self.ok).grid(row=3, column=0, sticky='w')
        Button(top, text='Cancel', command=self.cancel).grid(row=3, column=1, sticky='e')
        self.labelListbox.bind('<Double-Button>', self.selection)
        self.label_entry.bind('<Return>', self.ok)
        
    def selection(self, event): 
        widget = event.widget
        selection = widget.curselection()
        self.label = self.labelList[selection[0]]
        self.label_entry.delete('0', 'end')
        self.label_entry.insert('0', self.label)

    def ok(self, event=None): 
        id_inputchar = self.id_entry.get()
        if id_inputchar =='': 
            messagebox.showinfo(title='Warning', message='Please input object ID ')
            self.id_entry.focus_set()
            return
        try: 
            self.ObjectID = int(id_inputchar)
        except: 
            messagebox.showinfo(title='Warning', message='Object ID should be a number')
            self.id_entry.focus_set()
            return

        label_input = str(self.label_entry.get())
        if label_input == '': 
            messagebox.showinfo('Info','Please give label by inputting or double-clicking on list')
            return 
        if not label_input in self.labelList: 
            MsgBox = messagebox.askquestion('New Label', 'New label found! Do you want to add to label list')
            if MsgBox == 'yes': 
                self.labelList.append(label_input)
                self.label = label_input 
                self.destroy()
            else: 
                self.label = None
                self.label_entry['text'] = ''
                self.label_entry.focus_set()
        else: 
            self.label = label_input
            self.destroy()
       # print('label', self.label)
       # print('id', self.ObjectID)
    def cancel(self): 
        self.label = None
        self.destroy()

class ImageViewerFrame(ttk.Frame): 
    def __init__(self, mainframe, **opt):
        self.width = opt['width']
        self.height = opt['height']
        self.image_num = opt['image_num']
        ttk.Frame.__init__(self, master=mainframe, width=self.width, height=self.height,
                          borderwidth=3)
        
       # self.vbar = Scrollbar(self, orient='vertical')
      #  self.vbar.grid(row=0, column=1, sticky='ns')
        self.canvas = Canvas(self, highlightthickness=0,
                             width = self.width, height = self.height,
                             background='white')#yscrollcommand=self.vbar.set,
        self.canvas.grid(row=0, column=0, sticky='nswe')
 
        self.btns = [] 
        self.tkimages = []
        self.margin = 7
        self.column_num = int(math.ceil(math.sqrt(self.image_num))) #suppose NxN canvas 
        self.row_num = int(math.ceil(self.image_num/self.column_num))

        self.grid_size_w = int((self.width-(self.column_num+1)*self.margin)/self.column_num) 
        self.grid_size_h = int((self.height-(self.row_num+1)*self.margin)/self.row_num) 
        self.data = [] #each item is a dict:with keys {'frame_id', 'image', 'roi'}
        self.is_selected = np.zeros(self.image_num, dtype=bool)
        self.temp_prev_press_id = -1
        #print('grid size:', self.grid_size_w, self.grid_size_h)

    def get_thumbnail_shape(self): 
        return (self.grid_size_w, self.grid_size_h)

    def insertImage(self, data_dict): 
        self.data.append(data_dict)
        bgr = data_dict['image']
        idx = len(self.tkimages)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.grid_size_w, self.grid_size_h))
       # pil_img = Image.fromarray(rgb)
       # pil_img.thumbnail((self.grid_size_w, self.grid_size_h),Image.ANTIALIAS)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
       # label = Label(self, image=img)
        r,c = idx//self.column_num, idx%self.column_num
        x,y = c*self.grid_size_w+(c+1)*self.margin, r*self.grid_size_h +(r+1)*self.margin
        btn = Button(self, image=img, relief=FLAT)
        self.canvas.create_window(x,y, anchor=NW, window=btn)
        
        btn.bind("<Button-1>", lambda event, number=idx:self.normal_press(number))
        btn.bind("<Shift-Button-1>", lambda event, number=idx:self.shift_press(number))

        self.btns.append(btn)
       # self.label_objs.append(label)
        self.tkimages.append(img)

    def normal_press(self, number, reset=True):
        self.is_selected[number]  = not self.is_selected[number]
        if self.is_selected[number]: 
            self.btns[number].configure(bg='blue')
            self.btns[number].configure(activebackground='blue')
        else: 
            self.btns[number].configure(bg='white')
            self.btns[number].configure(activebackground='white')            

        if reset: 
            self.temp_prev_press_id = -1
    def shift_press(self, number): 
        
        if self.temp_prev_press_id == -1: 
            self.temp_prev_press_id = number
            self.normal_press(number, reset=False)
        else: 
            if number >= self.temp_prev_press_id:
                step = 1
            else: 
                step = -1
    
            for i in range(self.temp_prev_press_id+step, number+step,step): 
                self.normal_press(i, reset=False) 
            self.temp_prev_press_id = -1

    def select_all(self, do_select): 
        if do_select: 
            for i, btn in enumerate(self.btns): 
                self.is_selected[i] =True
                btn.configure(bg='blue')
                btn.configure(activebackground='blue')
        else: #clean all 
            for i, btn in enumerate(self.btns): 
                self.is_selected[i] =False
                btn.configure(bg='white')
                btn.configure(activebackground='white')


class View_AskAutoInsertion(Toplevel): 
    #auto search objects in later frames 
    def __init__(self, master,ttl_images, stop_event, width=600, height=500): 
        top = self.top = self
        self.width= width
        self.height = height
        self.ttl_images = int(ttl_images) #total frame number, used to set progressbar 
        Toplevel.__init__(self, master, width=self.width)
        x = master.winfo_x()
        y = master.winfo_y()
        self.geometry('{}x{}+{}+{}'.format(width,height, x+200,y+50))
        self.protocol("WM_DELETE_WINDOW",self.destroy)
        self.attributes('-topmost',True)
        self.transient()
        self.resizable(False, False)
        self.selectAllVar = IntVar()
        self.data_num = 0
        self.stop_event = stop_event
        self.title_label = Label(top, text="Searching Object in following frames")
        self.title_label.grid(row=0, column=0, sticky='we')
        self.progress_var = DoubleVar()
        self.progress_bar = ttk.Progressbar(top, variable=self.progress_var, maximum=100,
                                    orient=HORIZONTAL, length=self.width-80)
        self.progress_bar.grid(row=1, column=0,padx=5, sticky='w')
        self.stop_btn = Button(top, text='stop', command=self.stop_tracking)
        self.stop_btn.grid(row=1, column=1, sticky='e')

        self.thumbView = ImageViewerFrame(top, width=self.width-30, height=self.height-20, image_num=self.ttl_images)
        self.thumbView.grid(row=2, column=0, columnspan=2,padx=5, sticky='wens') 
        #self.thumbView.grid_propagate(0)
        #self.thumbView.update()

        self.selectallBtn = Checkbutton(top, variable = self.selectAllVar, 
                                        onvalue=1, offvalue=0, text="Select All", command= self.select_all)
        self.selectallBtn.grid(row=3, column=0, sticky='w')

        self.done_btn = Button(top, text="Done", command=self.select)
        self.done_btn.grid(row=4, column=0, sticky='e')
        self.cancel_btn = Button(top, text="Cancel", command=self.cancel)
        self.cancel_btn.grid(row=4,column=1, sticky='e')
        self.done_btn['state']='disable'
        self.cancel_btn['state'] = 'disable'
        self.rowconfigure(2, weight=1)
        
        self.done = False
        
    def get_thumbnail_shape(self): 
        return self.thumbView.get_thumbnail_shape()
    
    def insert(self, bgr): 
       # if stop_signal: 
       #     return
        self.data_num += 1
        self.progress_var.set(round(self.data_num/self.ttl_images*100))
        self.thumbView.insertImage(bgr)
        if self.data_num == self.ttl_images: 
            self.done_btn['state']='normal'
            self.cancel_btn['state'] = 'normal'
            self.stop_btn['state'] = 'disable'
            global stop_signal
            stop_signal= True
            self.title_label.config(text="Select on frames")

    def select(self): 
        
        self.done = True
        self.thumbView.destroy()
        self.destroy()
    
    def cancel(self): 
        self.done = False
        self.thumbView.destroy()
        self.destroy()
        
    def stop_tracking(self): 
       # global stop_signal
       # stop_signal =True
        self.stop_event.set()
        self.done_btn['state']='normal'
        self.cancel_btn['state'] = 'normal'
        self.stop_btn['state'] = 'disable'
        self.title_label.config(text="Select on frames")
        
    def select_all(self): 
        bselected = self.selectAllVar.get()
        if bselected: 
            self.thumbView.select_all(True)
        else: 
            self.thumbView.select_all(False)

