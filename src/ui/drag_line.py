
class DragLine(): 
    def __init__(self, parent, canvas, points, objectID,
                color, title = '',
                line_thickness=3, 
                tag='parallel_line',
                text_offset =[15,-15],
                radius=5):
        self.parent = parent
        self.previous_x = None
        self.previous_y = None
        self.selected = None
        self.objectID = objectID
        self.canvas = canvas
        self.freezeMode = False
        self.color = color 
        self.line_thickness = line_thickness
        self.tag = tag
        self.radius = radius
        self.title = title
        self.text_offset = text_offset
        self.text_font = ('Helvetica', 12)     

        #need to given two end points to draw a line 
        self.points = [[int(points[0]), int(points[1])], 
                       [int(points[2]), int(points[3])]]

        self.line = canvas.create_line(self.points[0][0], self.points[0][1],
                                        self.points[1][0],self.points[1][1],
                                        width=self.line_thickness, fill=self.color, 
                                        tag=self.tag, activefill=self.color)
        
        self.text = self.canvas.create_text(self.points[0][0]+self.text_offset[0], self.points[0][1]+self.text_offset[1], 
                                            text=self.title, fill=self.color, font=self.text_font,tag=self.tag)

    
        self.canvas.tag_bind(self.line, '<ButtonPress-1>',   lambda event, tag=self.line: self.on_press_tag(event, 0, tag))
        self.canvas.tag_bind(self.line, '<ButtonRelease-1>', lambda event, tag=self.line: self.on_release_tag(event, 0, tag))
        self.canvas.tag_bind(self.line, '<B1-Motion>', self.on_move_line)
        
        self.nodes = []
        for number, point in enumerate(self.points): 
            x,y = point[0], point[1]
            node = canvas.create_oval(x-self.radius, y-self.radius, x+self.radius, y+self.radius, 
                            fill=self.color, outline= 'black', tag=self.tag)
            self.nodes.append(node)
            self.canvas.tag_bind(node, '<ButtonPress-1>',   lambda event, number=number, tag=node: self.on_press_tag(event, number, tag))
            self.canvas.tag_bind(node, '<ButtonRelease-1>', lambda event, number=number, tag=node: self.on_release_tag(event, number, tag))
            self.canvas.tag_bind(node, '<B1-Motion>', lambda event, number=number: self.on_move_node(event, number))
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
            self.canvas.coords(self.line, coords)
            if number ==0: 
                text_coords[0] += self.text_offset[0]
                text_coords[1] += self.text_offset[1]
                self.canvas.coords(self.text, text_coords)

            self.previous_x = self.canvas.canvasx(event.x)#event.x
            self.previous_y = self.canvas.canvasy(event.y)#event.y
            self.parent.update_line(self.objectID, self.tag)

    def on_move_line(self, event): 
        if self.freezeMode: 
            return
        if self.selected:
            dx = self.canvas.canvasx(event.x)-self.previous_x#event.x - self.previous_x
            dy = self.canvas.canvasy(event.y)-self.previous_y#event.y - self.previous_y
            self.canvas.move(self.selected, dx, dy)
             #move text
            self.canvas.move(self.text, dx, dy)
            # move corner nodes 
            for item in self.nodes:
                self.canvas.move(item, dx, dy)
            for item in self.points:
                item[0] += dx
                item[1] += dy
            self.previous_x = self.canvas.canvasx(event.x)#event.x
            self.previous_y = self.canvas.canvasy(event.y)#eve
           
            self.parent.update_line(self.objectID, self.tag)
                        
    def erase(self): 
        self.canvas.delete(self.line)
        self.canvas.delete(self.text)
        for node in self.nodes: 
            self.canvas.delete(node)
    def freeze(self, freezable): 
        self.freezeMode = freezable

    def set_title(self, new_str): 
        self.canvas.itemconfig(self.text, text=new_str)
        self.title = new_str




class DragPolygon(): 
    def __init__(self, parent, canvas, points, objectID,
                line_color, node_color,
                title = '',
                line_thickness=3, 
                tag='line',
                text_offset =[15,-15],
                radius=5):
        self.parent = parent
        self.previous_x = None
        self.previous_y = None
        self.selected = None
        self.objectID = objectID
        self.canvas = canvas
        self.freezeMode = False
        self.line_color = line_color 
        self.node_color = node_color
        self.line_thickness = line_thickness
        self.tag = tag
        self.radius = radius
        self.title = title
        self.text_offset = text_offset
        self.text_font = ('Helvetica', 12)     

        #list of polygon nodes [(x1,y1),(x2,y2)...]
        self.points = points 
        self.point_num = len(self.points)

        self.lines =[]

        for i in range(self.point_num-1): 
            line= canvas.create_line(self.points[i][0], self.points[i][1],
                                     self.points[i+1][0],self.points[i+1][1],
                                      width=self.line_thickness, fill=self.line_color, 
                                      tag=self.tag, activefill=self.line_color)
            self.lines.append(line)
            self.canvas.tag_bind(line, '<ButtonPress-1>',   lambda event, number=i, tag=line: self.on_press_tag(event, number, tag))
            self.canvas.tag_bind(line, '<ButtonRelease-1>', lambda event, number=i, tag=line: self.on_release_tag(event, number, tag))
            self.canvas.tag_bind(line, '<B1-Motion>', lambda event, number=i: self.on_move_line(event, number))
        
        self.text = self.canvas.create_text(self.points[0][0]+self.text_offset[0], self.points[0][1]+self.text_offset[1], 
                                            text=self.title, fill=self.node_color, font=self.text_font,tag=self.tag)
   
        self.nodes = []
        for number, point in enumerate(self.points): 
            x,y = point[0], point[1]
            node = canvas.create_oval(x-self.radius, y-self.radius, x+self.radius, y+self.radius, 
                            fill=self.node_color, outline= 'black', tag=self.tag)
            self.nodes.append(node)
            self.canvas.tag_bind(node, '<ButtonPress-1>',   lambda event, number=number, tag=node: self.on_press_tag(event, number, tag))
            self.canvas.tag_bind(node, '<ButtonRelease-1>', lambda event, number=number, tag=node: self.on_release_tag(event, number, tag))
            self.canvas.tag_bind(node, '<B1-Motion>', lambda event, number=number: self.on_move_node(event, number))
    
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
            coords = self.canvas.coords(self.nodes[number])
            self.points[number][0] = int((coords[0]+coords[2])/2)
            self.points[number][1] = int((coords[1]+coords[3])/2)
            #update line 
            if number ==0: #starting point
                self.canvas.coords(self.lines[number], 
                                   self.points[0][0], self.points[0][1], 
                                   self.points[1][0], self.points[1][1])
            elif number == self.point_num-1: #ending point 
                self.canvas.coords(self.lines[number-1], 
                                   self.points[-2][0], self.points[-2][1], 
                                   self.points[-1][0], self.points[-1][1])
            else:
                self.canvas.coords(self.lines[number-1], 
                                   self.points[number-1][0], self.points[number-1][1], 
                                   self.points[number][0], self.points[number][1]) 
                self.canvas.coords(self.lines[number], 
                                   self.points[number][0], self.points[number][1], 
                                   self.points[number+1][0], self.points[number+1][1]) 

            # change points in polygons
            if number ==0: 
                text_coords = coords[2:4]
                text_coords[0] = max(coords[0], coords[2]) + self.text_offset[0]
                text_coords[1] = min(coords[1], coords[3]) + self.text_offset[1]
                self.canvas.coords(self.text, text_coords)

            self.previous_x = self.canvas.canvasx(event.x)#event.x
            self.previous_y = self.canvas.canvasy(event.y)#event.y
            self.parent.update_line(self.objectID)

    def on_move_line(self, event, number): 
        if self.freezeMode: 
            return
        if self.selected:
            dx = self.canvas.canvasx(event.x)-self.previous_x#event.x - self.previous_x
            dy = self.canvas.canvasy(event.y)-self.previous_y#event.y - self.previous_y
            #self.canvas.move(self.selected, dx, dy)#move line 
            #move releated nodes 
            self.canvas.move(self.nodes[number], dx, dy)
            self.canvas.move(self.nodes[number+1], dx,dy)

            self.points[number][0] += dx
            self.points[number][1] += dy
            self.points[number+1][0] += dx
            self.points[number+1][1] += dy

            for i in range(self.point_num-1): 
                self.canvas.coords(self.lines[i], self.points[i][0], self.points[i][1],
                                   self.points[i+1][0], self.points[i+1][1],)
            if number ==0: 
                text_x = self.points[0][0] + self.text_offset[0]
                text_y = self.points[0][1] + self.text_offset[1]
                self.canvas.coords(self.text, text_x, text_y)

            self.previous_x = self.canvas.canvasx(event.x)#event.x
            self.previous_y = self.canvas.canvasy(event.y)#eve
            
            self.parent.update_line(self.objectID)
                        
    def erase(self): 
        for line in self.lines: 
            self.canvas.delete(line)
        self.canvas.delete(self.text)
        for node in self.nodes: 
            self.canvas.delete(node)
    def freeze(self, freezable): 
        self.freezeMode = freezable

    def set_title(self, new_str): 
        self.canvas.itemconfig(self.text, text=new_str)
        self.title = new_str

