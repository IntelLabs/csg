import xml.etree.ElementTree as ET
import numpy as np
import math
from scipy import optimize
from sklearn.metrics import mean_squared_error

def prettyXml(element, indent, newline, level = 0): # elemnt为传进来的Elment类，参数indent用于缩进，newline用于换行  
    if element:  # 判断element是否有子元素  
        if element.text == None or element.text.isspace(): # 如果element的text没有内容  
            element.text = newline + indent * (level + 1)    
        else:  
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)  
    #else:  # 此处两行如果把注释去掉，Element的text也会另起一行  
        #element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * level  
    temp = list(element) # 将elemnt转成list  
    for subelement in temp:  
        if temp.index(subelement) < (len(temp) - 1): # 如果不是list的最后一个元素，说明下一个行是同级别元素的起始，缩进应一致  
            subelement.tail = newline + indent * (level + 1)  
        else:  # 如果是list的最后一个元素， 说明下一行是母元素的结束，缩进应该少一个  
            subelement.tail = newline + indent * level  
        prettyXml(subelement, indent, newline, level = level + 1) # 对子元素进行递归操作 

def LaneNode(lane_id,lane_type,isBelongToCent,lane_width,lane_dir,mark_type,mark_width,mark_color):
    #laneNode for straight road
    '''
    if lane_direction:
        lane_dir="forward"
    else:
        lane_dir="backward"
    '''
    lane_node=ET.Element('lane',id=str(lane_id),level='false',type=lane_type)
    if isBelongToCent==0:
        width=ET.SubElement(lane_node,"width",a=str(lane_width),b="0",c="0",d="0",sOffset="0.0000000000000000e+0")
        roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",type=mark_type, material="standard", color=mark_color  ,laneChange="none")
        userData=ET.SubElement(roadMark,"userData")
        vectorLane=ET.SubElement(userData,"vectorLane",travelDir=lane_dir)
    else:
        roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",material="standard", type="solid solid", width="1.2500000000000000e-1",color="yellow", laneChange="none")
    return lane_node


def LaneNode_rightturn(laneconnect,lane_id,lane_type,isBelongToCent,lane_width,lane_dir,mark_type,mark_width,mark_color):
    #lane_Node_link for connection road in junction

    lane_node=ET.Element('lane',id=str(lane_id),level='false',type=lane_type)
    if isBelongToCent==0:
        link=ET.SubElement(lane_node,"link")
        predecessor=ET.SubElement(link,"predecessor",id=str(laneconnect[0])) 
        successor=ET.SubElement(link,"successor",id=str(laneconnect[1])) 
        width=ET.SubElement(lane_node,"width",a=str(lane_width),b="0",c="0",d="0",sOffset="0.0000000000000000e+0")
        roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",type=mark_type, material="standard", color=mark_color  ,laneChange="none")
        userData=ET.SubElement(roadMark,"userData")
        vectorLane=ET.SubElement(userData,"vectorLane",travelDir=lane_dir)
    else:
        roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",type=mark_type, material="standard", color=mark_color  ,laneChange="none")
        #roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",material="standard", type="solid solid", width="1.2500000000000000e-1",color="yellow", laneChange="none")
    return lane_node


def RightTurnNode_new(predecessor_id,successor_id,hdgree,hdgree2,contact_Point,start_point, end_point):

    deltaX=abs(start_point[0]-end_point[0])
    deltaY=abs(start_point[1]-end_point[1])
    straight_length= abs(deltaX-deltaY)
    if predecessor_id[0]>successor_id[0]:
        xs=start_point[0]
        ys=start_point[1]
        xe=end_point[0]
        ye=end_point[1]
    else:
        xs=end_point[0]
        ys=end_point[1]
        xe=start_point[0]
        ye=start_point[1]
    
    if deltaX>deltaY:
        cur=-1/deltaY
        arc_length=abs(1/cur)*np.pi*2/4
        length1=arc_length
        length2=straight_length
        x1=xs
        y1=ys
        if hdgree2>=0:
            x2=xs+deltaY
        else:
            x2=xs-deltaY
        y2=ye
        curorder=1
        
    else:
        cur=-1/deltaX
        arc_length=abs(1/cur)*np.pi*2/4
        length1=straight_length 
        length2=arc_length
        x1=xs
        y1=ys
        x2=xs
        y2=ye-deltaX
        curorder=0
        
    s_length=length1+length2
    CRoad_id=successor_id[0]*10+predecessor_id[0]*9
    #arc_sign={'right_turn':-1.0,'straight':10000,'left_turn':1.0}
    #cur=-1/abs(start_point[0]-end_point[0])
    #arc_length=abs(1/cur)*np.pi*2/4
    CRoad_node=ET.Element('road',name="Road"+" "+str(CRoad_id) ,length=str(s_length), id=str(CRoad_id), junction="4")
    link=ET.SubElement(CRoad_node,'link')
    if predecessor_id[0]>successor_id[0]:
        predecessor=ET.SubElement(link,'predecessor',elementType="road", elementId=str(predecessor_id[0]),contactPoint=contact_Point[0]) 
        successor=ET.SubElement(link,'successor',elementType="road", elementId=str(successor_id[0]),contactPoint=contact_Point[1])
    else:
        predecessor=ET.SubElement(link,'predecessor',elementType="road", elementId=str(successor_id[0]),contactPoint=contact_Point[1]) 
        successor=ET.SubElement(link,'successor',elementType="road", elementId=str(predecessor_id[0]),contactPoint=contact_Point[0])
	
    type=ET.SubElement(CRoad_node,'type', s="0.0000000000000000e+0" ,type="town")
    planView=ET.SubElement(CRoad_node,'planView')
    if predecessor_id[0]>successor_id[0]:
        if curorder==1:
            geometry1=ET.SubElement(planView,'geometry',s='0',x=str(x1),y=str(y1),hdg=str(hdgree),length=str(length1))
            arc=ET.SubElement(geometry1,'arc',curvature=str(cur))
            geometry2=ET.SubElement(planView,'geometry',s=str(length1),x=str(x2),y=str(y2),hdg=str(hdgree2),length=str(length2))
            line=ET.SubElement(geometry2,'line')
        elif curorder==0:
            geometry1=ET.SubElement(planView,'geometry',s='0',x=str(x1),y=str(y1),hdg=str(hdgree),length=str(length1))
            line=ET.SubElement(geometry1,'line')
            geometry2=ET.SubElement(planView,'geometry',s=str(length1),x=str(x2),y=str(y2),hdg=str(hdgree),length=str(length2))
            arc=ET.SubElement(geometry2,'arc',curvature=str(cur))
            
    else:
        if curorder==1:
            geometry1=ET.SubElement(planView,'geometry',s='0',x=str(x1),y=str(y1),hdg=str(hdgree),length=str(length1))
            arc=ET.SubElement(geometry1,'arc',curvature=str(-cur))
            geometry2=ET.SubElement(planView,'geometry',s=str(length1),x=str(x2),y=str(y2),hdg=str(hdgree2),length=str(length2))
            line=ET.SubElement(geometry2,'line')
        elif curorder==0:
            geometry1=ET.SubElement(planView,'geometry',s='0',x=str(x1),y=str(y1),hdg=str(hdgree),length=str(length1))
            line=ET.SubElement(geometry1,'line')
            geometry2=ET.SubElement(planView,'geometry',s=str(length1),x=str(x2),y=str(y2),hdg=str(hdgree),length=str(length2))
            arc=ET.SubElement(geometry2,'arc',curvature=str(-cur))
        
    elevationProfile=ET.SubElement(CRoad_node,'elevationProfile')
    elevation1=ET.SubElement(elevationProfile,'elevation',s="0.0000000000000000e+0" ,a="0.0000000000000000e+0", b="0.0000000000000000e+0" ,c="0.0000000000000000e+0" ,d="0.0000000000000000e+0")
    elevation2=ET.SubElement(elevationProfile,'elevation',s=str(length1),a="0.0000000000000000e+0", b="0.0000000000000000e+0" ,c="0.0000000000000000e+0" ,d="0.0000000000000000e+0")
    lanes=ET.SubElement(CRoad_node,'lanes')
    laneOffset1=ET.SubElement(lanes,'laneOffset',s="0.0000000000000000e+0", a="0.0000000000000000e+0" ,b="0.0000000000000000e+0", c="0.0000000000000000e+0", d="0.0000000000000000e+0")
    laneOffset2=ET.SubElement(lanes,'laneOffset',s=str(length1), a="0.0000000000000000e+0" ,b="0.0000000000000000e+0", c="0.0000000000000000e+0", d="0.0000000000000000e+0")
    laneSection1=ET.SubElement(lanes,'laneSection',s="0")
    if predecessor_id[0]>successor_id[0]:
        center=ET.SubElement(laneSection1,'center')
        right=ET.SubElement(laneSection1,'right')
        lane1=LaneNode_rightturn([predecessor_id[1],-1],-1,'driving',0,3.5,"forward","none",0.2,"white")
        lane2=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        right.insert(0,lane1)
        center.insert(0,lane2)
    else:
        left=ET.SubElement(laneSection1,'left')
        center=ET.SubElement(laneSection1,'center')
        lane1=LaneNode_rightturn([successor_id[1],1],1,'driving',0,3.5,"backward","none",0.2,"white")
        lane2=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        left.insert(0,lane1)
        center.insert(1,lane2)

    laneSection2=ET.SubElement(lanes,'laneSection',s=str(length1))
    if predecessor_id[0]>successor_id[0]:
        center=ET.SubElement(laneSection2,'center')
        right=ET.SubElement(laneSection2,'right')
        lane1=LaneNode_rightturn([-1,successor_id[1]],-1,'driving',0,3.5,"forward","none",0.2,"white")
        lane2=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        right.insert(0,lane1)
        center.insert(0,lane2)
    else:
        left=ET.SubElement(laneSection2,'left')
        center=ET.SubElement(laneSection2,'center')
        lane1=LaneNode_rightturn([1,predecessor_id[1]],1,'driving',0,3.5,"backward","none",0.2,"white")
        lane2=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        left.insert(0,lane1)
        center.insert(1,lane2)
    return CRoad_node,CRoad_id
################################################left turn road######################################
def LaneNode_leftturn(laneconnect,lane_id,lane_type,isBelongToCent,lane_width,lane_dir,mark_type,mark_width,mark_color):
    lane_node=ET.Element('lane',id=str(lane_id),level='false',type=lane_type)
    if isBelongToCent==0:
        link=ET.SubElement(lane_node,"link")
        predecessor=ET.SubElement(link,"predecessor",id=str(laneconnect[0])) 
        successor=ET.SubElement(link,"successor",id=str(laneconnect[1])) 
        width=ET.SubElement(lane_node,"width",a=str(lane_width),b="0",c="0",d="0",sOffset="0.0000000000000000e+0")
        roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",type=mark_type, material="standard", color=mark_color  ,laneChange="none")
        userData=ET.SubElement(roadMark,"userData")
        vectorLane=ET.SubElement(userData,"vectorLane",travelDir=lane_dir)
    else:
        roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",type=mark_type, material="standard", color=mark_color  ,laneChange="none")
        #roadMark=ET.SubElement(lane_node,"roadMark",sOffset="0.0000000000000000e+0",material="standard", type="solid solid", width="1.2500000000000000e-1",color="yellow", laneChange="none")
    return lane_node
def LeftTurnNode_new(predecessor_id,successor_id,start_point,end_point,hdgree,contact_Point):
    #contact_Point=[]
    
    deltaX=abs(start_point[0]-end_point[0])
    deltaY=abs(start_point[1]-end_point[1])
    if deltaX>deltaY:
        cur=-1/deltaY
    else:
        cur=-1/deltaX
    length2=abs(1/cur)*np.pi*2/4
    length1= abs(deltaX-deltaY)
    s_length=length2+length1
    CRoad_id=successor_id[0]*100+predecessor_id[0]*7
    #CRoad_id=100
    if (successor_id[0]==2 and predecessor_id[0]==1) or(successor_id[0]==3 and predecessor_id[0]==0):
        cur=-abs(cur)
    else:
        cur=abs(cur)
    
    CRoad_node=ET.Element('road',name="Road"+" "+str(CRoad_id) ,length=str(s_length), id=str(CRoad_id), junction="4")
    link=ET.SubElement(CRoad_node,'link')
    predecessor=ET.SubElement(link,'predecessor',elementType="road", elementId=str(predecessor_id[0]),contactPoint=contact_Point[0]) 
    successor=ET.SubElement(link,'successor',elementType="road", elementId=str(successor_id[0]),contactPoint=contact_Point[1])
    planView=ET.SubElement(CRoad_node,'planView')
    
    geometry1=ET.SubElement(planView,'geometry',s='0',x=str(start_point[0]),y=str(start_point[1]),hdg=str(hdgree),length=str(length1))
    line=ET.SubElement(geometry1,'line')
    #arc1=ET.SubElement(geometry,'arc',curvature=str(cur))
    if predecessor_id[0]==0:
        x2=(start_point[0]+abs(deltaX-deltaY))
    else:
        x2=(start_point[0]-abs(deltaX-deltaY))
    print(x2)
    geometry2=ET.SubElement(planView,'geometry',s=str(length1),x=str(x2),y=str(start_point[1]),hdg=str(hdgree),length=str(length2))
    arc2=ET.SubElement(geometry2,'arc',curvature=str(cur))
    
    elevationProfile=ET.SubElement(CRoad_node,'elevationProfile')
    elevation1=ET.SubElement(elevationProfile,'elevation',s="0.0000000000000000e+0" ,a="0.0000000000000000e+0", b="0.0000000000000000e+0" ,c="0.0000000000000000e+0" ,d="0.0000000000000000e+0")
    elevation2=ET.SubElement(elevationProfile,'elevation',s=str(length1) ,a="0.0000000000000000e+0", b="0.0000000000000000e+0" ,c="0.0000000000000000e+0" ,d="0.0000000000000000e+0")

    lanes=ET.SubElement(CRoad_node,'lanes')
    laneOffset1=ET.SubElement(lanes,'laneOffset',s="0.0000000000000000e+0", a="0.0000000000000000e+0" ,b="0.0000000000000000e+0", c="0.0000000000000000e+0", d="0.0000000000000000e+0")
    laneOffset2=ET.SubElement(lanes,'laneOffset',s=str(length1), a="0.0000000000000000e+0" ,b="0.0000000000000000e+0", c="0.0000000000000000e+0", d="0.0000000000000000e+0")
    if (successor_id[0]==3 and predecessor_id[0]==1) or(successor_id[0]==2 and predecessor_id[0]==0):
        if predecessor_id[0]==1:
            s_l=1
            e_l=-1
        else:
            s_l=-1
            e_l=1
        laneSection1=ET.SubElement(lanes,'laneSection',s="0")
        center1=ET.SubElement(laneSection1,'center')
        right1=ET.SubElement(laneSection1,'right')
        lane11=LaneNode_rightturn([s_l,-1],-1,'driving',0,3.5,"forward","none",0.2,"white")
        lane21=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        right1.insert(0,lane11)
        center1.insert(0,lane21)

        laneSection2=ET.SubElement(lanes,'laneSection',s=str(length1))
        center2=ET.SubElement(laneSection2,'center')
        right2=ET.SubElement(laneSection2,'right')
        lane12=LaneNode_rightturn([-1,e_l],-1,'driving',0,3.5,"forward","none",0.2,"white")
        lane22=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        right2.insert(0,lane12)
        center2.insert(0,lane22)
    else:
        if predecessor_id[0]==1:
            s_l=-1
            e_l=-1
        else:
            s_l=1
            e_l=1
        laneSection1=ET.SubElement(lanes,'laneSection',s="0")
        left1=ET.SubElement(laneSection1,'left')
        center1=ET.SubElement(laneSection1,'center')
        
        lane11=LaneNode_rightturn([s_l,1],1,'driving',0,3.5,"forward","none",0.2,"white")
        lane21=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        left1.insert(0,lane11)
        center1.insert(0,lane21)

        laneSection2=ET.SubElement(lanes,'laneSection',s=str(length1))
        left2=ET.SubElement(laneSection2,'left')
        center2=ET.SubElement(laneSection2,'center')
        lane12=LaneNode_rightturn([1,e_l],1,'driving',0,3.5,"forward","none",0.2,"white")
        lane22=LaneNode_rightturn([0,0],0,'none',1,0,"forward","none",0.1,"white")
        left2.insert(0,lane12)
        center2.insert(0,lane22)

    return CRoad_node,CRoad_id

#################################################################J    
def Junction(connectionRoad_IDs,incomingRoad_IDs,laneconnect):
    
    JunctionNode=ET.Element('junction',id="4" ,name="junction4")
    #print(laneconnect)
    for f in range(4):
        if incomingRoad_IDs[f]==2 or 3:
            connecting_road='start'
        else:
            connecting_road='end'
        
        connection=ET.Element('connection',id=str(f), incomingRoad=str(incomingRoad_IDs[f]) ,connectingRoad=str(connectionRoad_IDs[f]), contactPoint=connecting_road)
        lanelink=ET.SubElement(connection,'laneLink',fro=str(laneconnect[f][0]), to=str(laneconnect[f][1]))
        JunctionNode.insert(f+1,connection)
    left_incomingRoad_IDs=[1,1,0,0]
    left_connectionRoad_IDs=[307,207,200,300]
    left_laneconnect=[[1,-1],[-1,1],[1,-1],[1,1]]
    for f in range(4):
        connection=ET.Element('connection',id=str(f+4), incomingRoad=str(left_incomingRoad_IDs[f]) ,connectingRoad=str(left_connectionRoad_IDs[f]), contactPoint='start')
        lanelink=ET.SubElement(connection,'laneLink',fro=str(left_laneconnect[f][0]), to=str(left_laneconnect[f][1]))
        JunctionNode.insert(f+5,connection)

    return JunctionNode    
def MainRoadNode(road_id,road_length,far_near_end,start_point,driving_lane_count,side_walk_count):
    road_node=ET.Element('road',name="Road"+" "+str(road_id) ,length=str(road_length), id=str(road_id), junction="-1")
    link=ET.SubElement(road_node,'link')
    if far_near_end=='far':
        successor=ET.SubElement(link,'successor',elementType="junction" ,elementId="4")
    else:
        predecessor=ET.SubElement(link,'predecessor',elementType="junction" ,elementId="4")
    type=ET.SubElement(road_node,'type', s="0.0000000000000000e+0" ,type="town")
    speed_type=ET.SubElement(type,'speed', max="40" ,unit="mph")
    planView=ET.SubElement(road_node,'planView')
    geometry=ET.SubElement(planView,'geometry',s='0.0000000000000000e+0',x=str(start_point[0]),y=str(start_point[1]),hdg=str(start_point[2]),length=str(road_length))
    line=ET.SubElement(geometry,'line')
    elevationProfile=ET.SubElement(road_node,'elevationProfile')
    elevation=ET.SubElement(elevationProfile,'elevation',s="0.0000000000000000e+0" ,a="0.0000000000000000e+0", b="0.0000000000000000e+0" ,c="0.0000000000000000e+0" ,d="0.0000000000000000e+0")
    lanes=ET.SubElement(road_node,'lanes')
    laneOffset=ET.SubElement(lanes,'laneOffset',s="0.0000000000000000e+0", a="0.0000000000000000e+0" ,b="0.0000000000000000e+0", c="0.0000000000000000e+0", d="0.0000000000000000e+0")
    laneSection=ET.SubElement(lanes,'laneSection',s="0.0000000000000000e+0")
    left=ET.SubElement(laneSection,'left')  
    center=ET.SubElement(laneSection,'center')
    center_lane=LaneNode(0,'none',1,0.125,"solid","solid",0.125,"white")
    center.insert(0,center_lane)
    right=ET.SubElement(laneSection,'right')   
    if driving_lane_count[0]>0:
        for f in reversed(range(1,driving_lane_count[0]+1)):
            lane=LaneNode(f,'driving',0,3.5,"backward","broken",0.2,"white")
            order=(driving_lane_count[0]+1-f)
            left.insert(order,lane)
    if driving_lane_count[1]>0:
        for f in range(1,driving_lane_count[1]+1):
            lane=LaneNode(-f,'driving',0,3.5,"forward","broken",0.2,"white")
            right.insert(f,lane)
    
    if side_walk_count[0]>0:
        for f in reversed(range(1+driving_lane_count[0],side_walk_count[0]+1+driving_lane_count[0])):
            lane=LaneNode(f,'sidewalk',0,3.0,"undirected","none",0.2,"white")
            order=side_walk_count[0]+1+driving_lane_count[0]-f
            left.insert(order,lane)
    if side_walk_count[1]>0:
        for f in range(1+driving_lane_count[1],side_walk_count[1]+1+driving_lane_count[1]):
            lane=LaneNode(-f,'sidewalk',0,3.0,"undirected","none",0.2,"white")
            right.insert(f,lane)   
    #print(road_node.attrib)
    return road_node

def piecewise_v1(x,x0,y0,k0,k1,k2):
    return np.piecewise(x , [x <= x0, x>x0] ,
                        [lambda x:k0*(x-x0) + y0,
                         lambda x:k2*(x-x0)**2+k1*(x-x0)+y0])


def piecewise_v2(x,x0,y0,k0,k1,k2):
    return np.piecewise(x , [x <= x0, x>x0] ,
                        [lambda x:k2*(x-x0)**2+k1*(x-x0)+y0,
                         lambda x:k0*(x-x0) + y0])

def CurFit(sample_pts):
    from scipy import optimize
    x=sample_pts[:,0]
    y=sample_pts[:,1]
    iteration=1000
    
    perr_min1 = np.inf
    p_best1 = None
    perr_min2 = np.inf
    p_best2 = None
    for n in range(iteration):
        k = np.random.rand(5)*20
        p1 , e1 = optimize.curve_fit(piecewise_v1, x, y,p0=k,maxfev=50000)
        perr1 = np.sum(np.abs(y-piecewise_v1(x, *p1)))
        if(perr1 < perr_min1):
            perr_min1 = perr1
            p_best1 = p1

        p2 , e2 = optimize.curve_fit(piecewise_v2, x, y,p0=k,maxfev=50000)
        perr2 = np.sum(np.abs(y-piecewise_v2(x, *p2)))
        if(perr2 < perr_min2):
            perr_min2 = perr2
            p_best2 = p2

    if perr_min1<perr_min2:
        P=p_best1
        fitfunction=1
    else:
        P=p_best2
        fitfunction=2
        
    return P,fitfunction

def curve_len_cal(startp,endp,dt,p):

    t = np.arange(startp,endp, dt)
    x=t
    k2=p[4]
    #k3=p[4]
    y0=p[1]
    x0=p[0]
    k1=p[3]
    y = k2*(x-x0)**2+k1*(x-x0)+y0
    #y = k1*x+y0-k1*x0
    len_list = [] 
    len_list = [np.sqrt( (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 ) for i in range(1,len(t))]
    length = sum(len_list)
    
    return length
	
def CurLenCalculate(startPts,endPts,currad):
    
    stringLen=np.sqrt((startPts[0]-endPts[0])**2+(startPts[1]-endPts[1])**2)
    angle=2*math.asin(stringLen/2/currad)
    CurLen=currad*angle
    return CurLen


def RoadGeoConstruction(p_best,x_start,x_end,func_type):
    
    x_vec=[x_start,p_best[0],x_end]
    
    sample_n=1000  #total sample points
    seg_n=500  #section count
    pts_seg=int(sample_n/seg_n)  #points per section

    #xxx = np.linspace(x_start,x_end,sample_n)
    if func_type == 1:
        xxx = np.linspace(x_vec[1],x_vec[2],sample_n)
        yvals=piecewise_v1(xxx, *p_best)  # choose function type
        xxx_line=np.linspace(x_vec[0],x_vec[1],sample_n)
        yyy_line=piecewise_v1(xxx_line, *p_best)  
    else:
        xxx = np.linspace(x_vec[0],x_vec[1],sample_n)
        yvals=piecewise_v2(xxx, *p_best)
        xxx_line=np.linspace(x_vec[1],x_vec[2],sample_n)
        yyy_line=piecewise_v2(xxx_line, *p_best)
        
    kk=np.gradient(yvals)
    kkk=np.gradient(kk)
    
    hdg_line=math.atan(p_best[2])
    print(xxx[0])
    cur_list=np.zeros([1,seg_n])
    cur_len_list=np.zeros([1,seg_n])
    cur_start_list=np.ones([2,seg_n])
    hdg_cur = np.zeros([1,seg_n])
    for n in range(0,seg_n):
        cur_p=np.zeros([1,pts_seg])
        for i in range(pts_seg*n,pts_seg*(n+1)):
            cur_p[0,i%pts_seg]=((1 + (kk[i])**2)**1.5) / np.absolute(kkk[i])

        cur_list[0,n]=np.mean(1/cur_p[0,])
        startPts=[xxx[pts_seg*n],yvals[pts_seg*n]]
        endPts=[xxx[min(pts_seg*(n+1),sample_n-1)],yvals[min(pts_seg*(n+1),sample_n-1)]]
        currad=1/cur_list[0,n]
        cur_len_list[0,n]=CurLenCalculate(startPts,endPts,currad)
        hdg_cur[0,n]=math.atan(2*p_best[4]*xxx[pts_seg*n]+p_best[3])
        cur_start_list[0,n]=startPts[0]
        cur_start_list[1,n]=startPts[1]
        if n==0:
            print(startPts)
            print(cur_start_list[:,0])
    
    cur_len=curve_len_cal(xxx[0],xxx[-1],0.01,p_best)
    print(cur_len)
    
    line_start=[xxx_line[0],yyy_line[0]]
    line_len=np.sqrt((xxx_line[0]-xxx_line[-1])**2+(yyy_line[0]-yyy_line[-1])**2)
    #print(cur_l)
    return cur_list,cur_len_list,cur_start_list,hdg_cur,line_len,line_start,hdg_line

def CurRoadNode_new(road_id,far_near_end,driving_lane_count,side_walk_count,function_flag,cur_array, cur_len, cur_position, cur_hdg, line_len, line_start, line_hdg):
    #_,seg_list = CalculateCur(road_samples,2,30)
    
    road_length=np.sum(cur_len)+line_len
    road_node=ET.Element('road',name="Road"+" "+str(road_id) ,length=str(road_length), id=str(road_id), junction="-1")
    link=ET.SubElement(road_node,'link')
    
    if far_near_end=='far':
        successor=ET.SubElement(link,'successor',elementType="junction" ,elementId="4")
    else:
        predecessor=ET.SubElement(link,'predecessor',elementType="junction" ,elementId="4")
    
    type=ET.SubElement(road_node,'type', s="0.0000000000000000e+0" ,type="town")
    speed_type=ET.SubElement(type,'speed', max="40" ,unit="mph")
    planView=ET.SubElement(road_node,'planView')
    s_start=0
    if function_flag ==1:
        x_num=line_start[0]
        y_num=line_start[1]
        seg_len=line_len
        hdg_val=line_hdg
        geometry=ET.SubElement(planView,'geometry',s=str(s_start),x=str(x_num),y=str(y_num),hdg=str(hdg_val),length=str(seg_len))
        line=ET.SubElement(geometry,'line')
        s_start=line_len
        for i in range(len(cur_array[0])):
            print(i)
            x_num=cur_position[0,i]
            y_num=cur_position[1,i]
            seg_len=cur_len[0][i]
            hdg_val=cur_hdg[0][i]
            geometry=ET.SubElement(planView,'geometry',s=str(s_start),x=str(x_num),y=str(y_num),hdg=str(hdg_val),length=str(seg_len))
            arc=ET.SubElement(geometry,'arc',curvature=str(cur_array[0][i]))
            s_start+=seg_len
            
    else:
        s_start=0
        for i in range(len(cur_array[0])):
            x_num=cur_position[0,i]
            y_num=cur_position[1,i]
            seg_len=cur_len[0][i]
            hdg_val=cur_hdg[0][i]
            geometry=ET.SubElement(planView,'geometry',s=str(s_start),x=str(x_num),y=str(y_num),hdg=str(hdg_val),length=str(seg_len))
            arc=ET.SubElement(geometry,'arc',curvature=str(cur_array[0][i]))
            s_start+=seg_len
                                   
        x_num=line_start[0]
        y_num=line_start[1]
        seg_len=line_len
        hdg_val=line_hdg
        geometry=ET.SubElement(planView,'geometry',s=str(s_start),x=str(x_num),y=str(y_num),hdg=str(hdg_val),length=str(seg_len))
        line=ET.SubElement(geometry,'line')      
    elevationProfile=ET.SubElement(road_node,'elevationProfile')
    elevation=ET.SubElement(elevationProfile,'elevation',s="0.0000000000000000e+0" ,a="0.0000000000000000e+0", b="0.0000000000000000e+0" ,c="0.0000000000000000e+0" ,d="0.0000000000000000e+0")
    lanes=ET.SubElement(road_node,'lanes')
    laneOffset=ET.SubElement(lanes,'laneOffset',s="0.0000000000000000e+0", a="0.0000000000000000e+0" ,b="0.0000000000000000e+0", c="0.0000000000000000e+0", d="0.0000000000000000e+0")
    laneSection=ET.SubElement(lanes,'laneSection',s="0.0000000000000000e+0")
    left=ET.SubElement(laneSection,'left')  
    center=ET.SubElement(laneSection,'center')
    center_lane=LaneNode(0,'none',1,0.125,"solid","solid",0.125,"white")
    center.insert(0,center_lane)
    right=ET.SubElement(laneSection,'right')   
    if driving_lane_count[0]>0:
        for f in reversed(range(1,driving_lane_count[0]+1)):
            lane=LaneNode(f,'driving',0,3.5,"backward","broken",0.2,"white")
            order=(driving_lane_count[0]+1-f)
            left.insert(order,lane)
    if driving_lane_count[1]>0:
        for f in range(1,driving_lane_count[1]+1):
            lane=LaneNode(-f,'driving',0,3.5,"forward","broken",0.2,"white")
            right.insert(f,lane)
    
    if side_walk_count[0]>0:
        for f in reversed(range(1+driving_lane_count[0],side_walk_count[0]+1+driving_lane_count[0])):
            lane=LaneNode(f,'sidewalk',0,3.0,"undirected","none",0.2,"white")
            order=side_walk_count[0]+1+driving_lane_count[0]-f
            left.insert(order,lane)
    if side_walk_count[1]>0:
        for f in range(1+driving_lane_count[1],side_walk_count[1]+1+driving_lane_count[1]):
            lane=LaneNode(-f,'sidewalk',0,3.0,"undirected","none",0.2,"white")
            right.insert(f,lane)   
    #print(road_node.attrib)
    return road_node

def replace(file_path, old_str, new_str):

    f = open(file_path,'r+')
    all_lines = f.readlines()
    f.seek(0)
    f.truncate()
    for line in all_lines:
      line = line.replace(old_str, new_str)
      f.write(line)
    f.close()
    
def GenerateRoad(sample_pts,filename,roadID,driving_lane_count,sidewalk_lane_count):
    rootNode=ET.Element('OpenDRIVE')
    headerNode=ET.SubElement(rootNode,'header',revMajor="1" ,revMinor="4" ,name="" ,version="1" ,date="2019-04-15T11:01:06" ,north="1.0059999847412109e+2", south="-9.9889999389648438e+1" ,east="1.1941999816894531e+2", west="-1.2104999542236328e+2", vendor="VectorZero")
    userData=ET.SubElement(headerNode,'userData')
    vectorScene=ET.SubElement(userData,'vectorScene',program="RoadRunner" ,version="2019.0.2 (build fcf98526c)")  
    # add main road coming to junction

    ppp1,function_flag=CurFit(sample_pts)
    Cur_array, Cur_len, Cur_position, Cur_hdg, Line_len, Line_start, Line_hdg= RoadGeoConstruction(ppp1,sample_pts[0,0],
                                                                                                   sample_pts[-1,0],
                                                                                                   function_flag)
    roadnode=CurRoadNode_new(roadID,driving_lane_count,sidewalk_lane_count,function_flag,
                             Cur_array,Cur_len, Cur_position, Cur_hdg, Line_len, Line_start, Line_hdg)
    #roadnode=CurRoadNode(0,sample_pts,[2,2],[1,1])
    rootNode.insert(1,roadnode)

    prettyXml(rootNode, '\t', '\n') #format normalization   
    #ET.dump(rootNode)  
    Newtree=ET.ElementTree(rootNode)
    #ET.dump(rootNode)  
    Newtree.write(filename)
    #replace(filename, 'fro', 'from')    
def GenerateIntersection(filename,Intersection_bb,road_len,driving_lane_counts,edit_flag,Cur_samples):
    #constant internal parameters
	#the order should be left, right, up and down.
	#Cur_samples=list[array1,array2,array3,array4]
    hdg_mat=np.array([0,0,-np.pi/2,-np.pi/2]).reshape((4,1))
    start_points=np.concatenate((Intersection_bb,hdg_mat),axis=1)
    start_points=np.concatenate((start_points,road_len),axis=1)
    start_points[0,0]-=road_len[0]
    start_points[2,1]+=road_len[2]
    print(start_points)
    SPts=start_points
    SPts_position=['far','near','far','near']
    RTOrder=[[3,1],[1,2],[2,0],[0,3]] #right turn order
    RTconnectPts=[['start','start'],['start','end'],['end','end'],['end','start']]
    RThdgree=[np.pi/2,-np.pi/2,-np.pi/2,np.pi/2]
    RThdgree2=[0,0,-np.pi,-np.pi]
    RTOrderArray=np.array([[3,1],[1,2],[2,0],[0,3]])

    #calculate parameters

    RTlaneconnect=[[driving_lane_counts[3,0],-driving_lane_counts[1,1]],[driving_lane_counts[1,1],driving_lane_counts[2,0]],[-driving_lane_counts[2,1],driving_lane_counts[0,0]],[-driving_lane_counts[0,1],-driving_lane_counts[3,1]]] #order in 3 1 2 0
    RTstartPts=Intersection_bb[[3,1,2,0],:]
    RTstartPts[0,0]+=(driving_lane_counts[3,0]-1)*3.5
    RTstartPts[1,1]+=(driving_lane_counts[1,0]-1)*3.5
    RTstartPts[2,0]-=(driving_lane_counts[2,1]-1)*3.5
    RTstartPts[3,1]-=(driving_lane_counts[0,0]-1)*3.5
    RTendPts=Intersection_bb[[1,2,0,3],:]
    RTendPts[0,1]-=(driving_lane_counts[1,0]-1)*3.5
    RTendPts[1,0]+=(driving_lane_counts[2,0]-1)*3.5
    RTendPts[2,1]+=(driving_lane_counts[0,0]-1)*3.5
    RTendPts[3,0]-=(driving_lane_counts[3,1]-1)*3.5
    print(RTstartPts)
    print(RTendPts)
    rootNode=ET.Element('OpenDRIVE')
    headerNode=ET.SubElement(rootNode,'header',revMajor="1" ,revMinor="4" ,name="" ,version="1" ,date="2019-04-15T11:01:06" ,north="1.0059999847412109e+2", south="-9.9889999389648438e+1" ,east="1.1941999816894531e+2", west="-1.2104999542236328e+2", vendor="VectorZero")
    userData=ET.SubElement(headerNode,'userData')
    vectorScene=ET.SubElement(userData,'vectorScene',program="RoadRunner" ,version="2019.0.2 (build fcf98526c)")  
    # add main road coming to junction
    for i in range(4):
        if i==edit_flag:
            sample_pts=Cur_samples
            ppp1,function_flag=CurFit(sample_pts)
            Cur_array, Cur_len, Cur_position, Cur_hdg, Line_len, Line_start, Line_hdg= RoadGeoConstruction(ppp1,sample_pts[0,0],
                                                                                                   sample_pts[-1,0],
                                                                                                   function_flag)
            roadnode=CurRoadNode_new(i,SPts_position[i],driving_lane_counts[i,:],[1,1],function_flag,
                             Cur_array,Cur_len, Cur_position, Cur_hdg, Line_len, Line_start, Line_hdg)
            rootNode.insert(i+1,roadnode)

        else:
            roadnode=MainRoadNode(i,SPts[i,3],SPts_position[i],SPts[i,0:3],driving_lane_counts[i,:],[1,1])
            rootNode.insert(i+1,roadnode)
    # add right turn road
    rightRoad_IDs=[]
    for i in range(4):
        rightnode,road_id=RightTurnNode_new([RTOrder[i][0],RTlaneconnect[i][0]],[RTOrder[i][1],RTlaneconnect[i][1]],RThdgree[i],RThdgree2[i],RTconnectPts[i][0:2],RTstartPts[i,:],RTendPts[i,:])
        print(road_id,RTstartPts[i,:],RTendPts[i,:])
        rootNode.insert(i+5,rightnode)
        rightRoad_IDs.append(road_id)

    # add left turn road
    leftRoad_IDs=[]
    Left_predecessor=[[1,0],[1,0],[0,0],[0,0]]
    Left_successor = [[3,0],[2,0],[2,0],[3,0]]
    Left_start_point=[Intersection_bb[1],Intersection_bb[1],Intersection_bb[0],Intersection_bb[0]]
    Left_end_point=[Intersection_bb[3],Intersection_bb[2],Intersection_bb[2],Intersection_bb[3]]
    Left_hdgree=[-np.pi,-np.pi,0,0]
    LTcontact_Point=RTconnectPts
    
    for i in range(4):
        leftnode,road_id=LeftTurnNode_new(Left_predecessor[i],Left_successor[i],Left_start_point[i],Left_end_point[i],Left_hdgree[i],LTcontact_Point[i])
        print(road_id,RTstartPts[i,:],RTendPts[i,:])
        rootNode.insert(i+9,leftnode)
        leftRoad_IDs.append(road_id)

        
    #leftnode,left_road_id=LeftTurnNode_new()
    #rootNode.insert(9,leftnode)
    
    LaneConnJun=[[driving_lane_counts[3,0],-1],[driving_lane_counts[2,0],1],[-driving_lane_counts[2,1],-1],[-driving_lane_counts[3,1],1]]
    print(LaneConnJun)
    JunctionNode=Junction(rightRoad_IDs,[3,2,2,3],LaneConnJun)
    rootNode.insert(13,JunctionNode)
    prettyXml(rootNode, '\t', '\n')            #format normalization   
    #ET.dump(rootNode)  
    Newtree=ET.ElementTree(rootNode)
    #ET.dump(rootNode)  
    Newtree.write(filename)
    replace(filename, 'fro', 'from')
	
	
	
	
	