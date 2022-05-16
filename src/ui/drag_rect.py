from src.ui.color import alpha_hex,scale_hex

class DragRect(): 
    def __init__(self, parent, canvas, points, color, title="",
                objectID=0, text_font=None):
        self.parent = parent
        self.previous_x = None
        self.previous_y = None
        self.selected = None
        self.color = color
        self.canvas = canvas
        self.title = title
        self.objectID = objectID #used to distinguish objects
        self.freezeMode = False #cannot move objects

        self.alpha = 0.2
        self.line_scale = 1.6
        self.radius = 5
        self.line_thickness = 3
        self.text_offset = [15,-15]
        self.text_color = color
        if text_font is None: 
            self.text_font = ('Helvetica', 12)
        else: 
            self.text_font = text_font

        #given 2 points, augment to 4 corner points
        self.points = points #len(points)==4
        self.fill_color = alpha_hex(color, alpha=self.alpha) #alpha blending
        self.line_color = scale_hex(color, scale=self.line_scale)

        self.text = self.canvas.create_text(self.points[0][0]+self.text_offset[0], self.points[0][1]+self.text_offset[1], 
                                            text=self.title, fill=self.text_color, font=self.text_font,tag='dragRect')

        self.polygon = canvas.create_rectangle(self.points, width=self.line_thickness, outline=self.line_color, 
                                               tag='dragRect', activefill=self.fill_color)
        self.canvas.tag_bind(self.polygon, '<ButtonPress-1>',   lambda event, tag=self.polygon: self.on_press_tag(event, 0, tag))
        self.canvas.tag_bind(self.polygon, '<ButtonRelease-1>', lambda event, tag=self.polygon: self.on_release_tag(event, 0, tag))
        self.canvas.tag_bind(self.polygon, '<B1-Motion>', self.on_move_rect)
        
        self.all_object_list = [self.polygon]
        self.all_object_list.append(self.text)

        # nodes - red rectangles
        self.nodes = []
        for number, point in enumerate(self.points):
            x, y = point
           # node = canvas.create_rectangle((x-self.radius, y-self.radius, x+self.radius, y+self.radius), fill=self.color, outline= self.line_color)
            node = canvas.create_oval(x-self.radius, y-self.radius, x+self.radius, y+self.radius, fill=self.color, 
                                    outline= self.line_color, tag='dragRect')
            self.nodes.append(node)
            self.canvas.tag_bind(node, '<ButtonPress-1>',   lambda event, number=number, tag=node: self.on_press_tag(event, number, tag))
            self.canvas.tag_bind(node, '<ButtonRelease-1>', lambda event, number=number, tag=node: self.on_release_tag(event, number, tag))
            self.canvas.tag_bind(node, '<B1-Motion>', lambda event, number=number: self.on_move_node(event, number))
            self.all_object_list.append(node)
        
    def on_press_tag(self, event, number, tag):
        self.selected = tag
        self.previous_x = self.canvas.canvasx(event.x)# event.x
        self.previous_y = self.canvas.canvasy(event.y)#event.y

    def on_release_tag(self, event, number, tag):
        self.selected = None
        self.previous_x = None
        self.previous_y = None
    #move single node
    def on_move_node(self, event, number):
        if self.freezeMode: 
            return
        if self.selected: 
            dx = self.canvas.canvasx(event.x)-self.previous_x#event.x - self.previous_x
            dy = self.canvas.canvasy(event.y)-self.previous_y#event.y - self.previous_y

            self.canvas.move(self.selected, dx, dy)

            #update all points, always binding to 2 corner points
            for i in range(2): 
                curr_coords = self.canvas.coords(self.nodes[i])
                self.points[i][0] = int((curr_coords[0]+curr_coords[2])/2)
                self.points[i][1] = int((curr_coords[1]+curr_coords[3])/2)
                if i==0: #always binding to the first point
                    text_coords = curr_coords[2:4]
                    text_coords[0] = max(curr_coords[0], curr_coords[2])
                    text_coords[1] = min(curr_coords[1], curr_coords[3])

            coords = sum(self.points, []) #flatten
            # change points in polygons
            self.canvas.coords(self.polygon, coords)
            
            if number ==0: 
                text_coords[0] += self.text_offset[0]
                text_coords[1] += self.text_offset[1]
                self.canvas.coords(self.text, text_coords)

            self.previous_x = self.canvas.canvasx(event.x)#event.x
            self.previous_y = self.canvas.canvasy(event.y)#event.y

            self.parent.update_dragrect(self.objectID)
    def on_move_rect(self, event): 
        if self.freezeMode: 
            return
        if self.selected:
            dx = self.canvas.canvasx(event.x)-self.previous_x#event.x - self.previous_x
            dy = self.canvas.canvasy(event.y)-self.previous_y#event.y - self.previous_y
            #dx = event.x - self.previous_x
            #dy = event.y - self.previous_y
            
            #move text
            self.canvas.move(self.text, dx, dy)

            # move polygon
            self.canvas.move(self.selected, dx, dy)

            # move corner nodes 
            for item in self.nodes:
                self.canvas.move(item, dx, dy)

            # recalculate values in self.points
            for item in self.points:
                item[0] += dx
                item[1] += dy

            self.previous_x = self.canvas.canvasx(event.x)#event.x
            self.previous_y = self.canvas.canvasy(event.y)#eve
           # self.previous_x = event.x
           # self.previous_y = event.y
            self.parent.update_dragrect(self.objectID)
    
    def erase(self): 
        self.canvas.delete(self.polygon)
        self.canvas.delete(self.text)
        for node in self.nodes: 
            self.canvas.delete(node)

    def set_title(self, new_str): 
        self.canvas.itemconfig(self.text, text=new_str)
        strss = new_str.split('  ')
        self.objectID = int(strss[1])
        self.title = new_str

    def freeze(self, freezable): 
        self.freezeMode = freezable

    def highlight(self, highlight=True): 
        if highlight: 
            self.canvas.itemconfig(self.polygon,fill=self.fill_color)
        else: 
            self.canvas.itemconfig(self.polygon, fill='')