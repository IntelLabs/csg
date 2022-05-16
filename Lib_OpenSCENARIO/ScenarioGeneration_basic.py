# coding: utf-8

# In[ ]:


import pyoscx
import sys
import os
import numpy as np
import xml.etree.ElementTree as ET
import argparse

def XMLparse(xml_filename):
    #tree = ET.parse('case2_TrafficDynamics.xml')
    tree = ET.parse(xml_filename)
    root = tree.getroot()
    scenario=root.getchildren()
    hero_name=[]
    #xinxin, support init velocity setting
    hero_initV=[]
    hero_pos_list=[]
    hero = scenario[0].findall("ego_vehicle")
    for actor in hero:
        actor_initial_pos=[float(actor.attrib['x']),float(actor.attrib['y']),float(actor.attrib['z']),float(actor.attrib['yaw'])]
        hero_name.append(actor.attrib['model'])
        #hero_initV.append(actor.attrib['velocity'])
        hero_initV.append(actor.attrib['velocity'] if 'velocity' in actor.attrib else 0)
        hero_pos_list.append(actor_initial_pos)

    hero_route = scenario[0].findall("route")[0].findall("waypoint")
    for actor in hero_route:
        hero_pos_list.append([float(actor.attrib['x']),float(actor.attrib['y']),float(actor.attrib['z'])])

    hero_target = scenario[0].findall("target")
    for actor in hero_target:
        actor_target_pos=[float(actor.attrib['x']),float(actor.attrib['y']),float(actor.attrib['z'])]
        hero_pos_list.append(actor_target_pos)
    #print(hero_pos_list)

    other_actors = scenario[0].findall("other_actor")
    actor_pos_list=[]
    actor_name_list=[]
    actor_initV_list=[]
    for actor in other_actors:
        #print(actor.attrib)
        actor_list=[]
        actor_initial_pos=[float(actor.attrib['x']),float(actor.attrib['y']),float(actor.attrib['z']),float(actor.attrib['yaw'])]
        actor_name_list.append(actor.attrib['model'])
        #xinxin
        #actor_initV_list.append(actor.attrib['velocity'])
        actor_initV_list.append(actor.attrib['velocity'] if 'velocity' in actor.attrib else 0)
        actor_list.append(actor_initial_pos)
        actor_route = actor.findall('other_route')
        if actor_route:
            #print(actor_route[0].tag)
            waypoint_list=actor_route[0].findall('waypoint')

            if waypoint_list:
                #print(waypoint_list[0].attrib)
                for waypoint in waypoint_list:
                    actor_list.append([float(waypoint.attrib['x']), float(waypoint.attrib['y']),float(waypoint.attrib['z'])]) 
            #print(actor_list)
        else:
            actor_list.append('None')
            #print(actor_list)
        actor_pos_list.append(actor_list)
    return hero_name,hero_initV, hero_pos_list, actor_name_list, actor_initV_list, actor_pos_list

def replace(file_path, old_str, new_str):

    f = open(file_path,'r+')
    all_lines = f.readlines()
    f.seek(0)
    f.truncate()
    for line in all_lines:
      line = line.replace(old_str, new_str)
      f.write(line)
    f.close()
    
def generateScenario(xml_filename,map_name,output_filename, controller):

    init_v_factor=2.0

    #define actor numbers
    hero_name, hero_initV, hero_pos_list, actor_name_list, actor_initV_list, actor_pos_list = XMLparse(xml_filename)
    actor_num=len(actor_pos_list)

#########################################
####  ParameterDeclarations
#########################################
    paramdec = pyoscx.ParameterDeclarations()
    #paramdec.add_parameter(pyoscx.Parameter('$HostVehicle',pyoscx.ParameterType.string,'car_white'))
    #paramdec.add_parameter(pyoscx.Parameter('$TargetVehicle',pyoscx.ParameterType.string,'car_red'))

#########################################
####  CatalogLocations
#########################################
    catalog = pyoscx.Catalog()
    catalog.add_catalog('VehicleCatalog','../xosc/Catalogs/Vehicles')

#########################################
####  RoadNetwork
#########################################
    #map_name='xxxxx'
    road = pyoscx.RoadNetwork(roadfile=map_name,scenegraph='')
    
    
#########################################
####  Entities
#########################################
    entities = pyoscx.Entities()

    bb = pyoscx.BoundingBox(2,5,1.8,2.0,0,0.9)
    fa = pyoscx.Axle(30,0.8,1.68,2.98,0.4)
    ba = pyoscx.Axle(30,0.8,1.68,0,0.4)

    #hero ScenarioObject
    hero_veh = pyoscx.Vehicle(hero_name[0],pyoscx.VehicleCategory.car,bb,fa,ba,69,10,10)
    hero_veh.add_property('type', 'ego_vehicle')
    hero_veh.add_property('color', '255,0,0')
    entities.add_scenario_object('hero',hero_veh)
    #NPC ScenarioObject
    for i in range(len(actor_name_list)):
        #print(len(all_wp_list[i]))
        actor_name='actor'+str(i)
        actor_veh = pyoscx.Vehicle(actor_name_list[i],pyoscx.VehicleCategory.car,bb,fa,ba,69,10,10)
        entities.add_scenario_object(actor_name,actor_veh)

#########################################
####  Storyboard
#########################################
    #Init
    init = pyoscx.Init()
    #Stroy
    storyparam = pyoscx.ParameterDeclarations()
    story = pyoscx.Story('mystory',storyparam)
    #StopTrigger
    sb_stoptrigger = pyoscx.ConditionGroup('stop')    

    sb = pyoscx.StoryBoard(init, sb_stoptrigger)
    sb.add_story(story)

    #====================================
    ####  Storyboard-->Init
    #====================================

    ###### Storyboard-->Init-->EnvironmentAction
    timeofday = pyoscx.TimeOfDay('false', 2010, 11, 20, 8, 30, 0)
    weather = pyoscx.Weather(cloudstate=pyoscx.CloudState.free, 
                             sun_intensity=0.85, sun_azimuth=0, sun_elevation=1.31,
                             precipitation=pyoscx.PrecipitationType.dry, precipitation_intensity=0)
                             
    roadcondition = pyoscx.RoadCondition(friction_scale_factor=1.0)
    env = pyoscx.Environment('Environment1',timeofday, weather, roadcondition)  
    EnvAction = pyoscx.EnvironmentAction('Environment1', env)
    init.add_global_action(EnvAction)

    #Storyboard-->Init-->Hero
    step_time = pyoscx.TransitionDynamics(pyoscx.DynamicsShapes.step,pyoscx.DynamicsDimension.time,1)
    hero_speed = pyoscx.AbsoluteSpeedAction(str(float(hero_initV[0])/init_v_factor),step_time)
    init.add_init_action('hero',hero_speed)	#add init speed
    
    hero_startp = pyoscx.TeleportAction(pyoscx.WorldPosition(round(hero_pos_list[0][0],2)-1,round(hero_pos_list[0][1],2)-1,round(hero_pos_list[0][2],2),round(np.deg2rad(hero_pos_list[0][3]),2)))
    init.add_init_action('hero',hero_startp) #add init position

    print('ego controller is', controller)
    hero_controller_prop = pyoscx.Properties()
    hero_controller_prop.add_property(name='module', value=controller)
    #added for simple control
    hero_controller_prop.add_property(name='attach_camera', value='true')
    hero_controller = pyoscx.Controller('HeroAgent', hero_controller_prop)
    #hero_controller_overrideaction = pyoscx.OverrideThrottleAction(value=0, activate=False)
    #init.add_init_action('hero',hero_controller_overrideaction)
    hero_controller_action = pyoscx.ControllerAction(hero_controller)
    init.add_init_action('hero', hero_controller_action)	#add controller 

    #Storyboard-->Init-->NPCs
    for i in range(len(actor_name_list)):
        actor_name='actor'+str(i)
        #speed = pyoscx.AbsoluteSpeedAction(15,step_time)
        speed = pyoscx.AbsoluteSpeedAction(str(float(actor_initV_list[i])/init_v_factor),step_time)
        startp = pyoscx.TeleportAction(pyoscx.WorldPosition(round(actor_pos_list[i][0][0],2),round(actor_pos_list[i][0][1],2),round(actor_pos_list[i][0][2],2),round(np.deg2rad(actor_pos_list[i][0][3]),2)))
        init.add_init_action(actor_name, startp)  #add npc position

        if len(actor_pos_list[i])>2:
            init.add_init_action(actor_name,speed)		#add npc speed


    #====================================
    ####  Storyboard-->Story
    #====================================

    ######  Storyboard-->Story-->ParameterDeclarations

    storyparam.add_parameter(pyoscx.Parameter('$owner',pyoscx.ParameterType.string,'hero'))


    ######  Storyboard-->Story-->Act
    act_start_trigger = pyoscx.ValueTrigger('Act_StartTrigger',0, pyoscx.ConditionEdge.rising, pyoscx.SimulationTimeCondition(0,pyoscx.Rule.greaterThan))
    #trig_cond1 = pyoscx.TimeToCollisionCondition(2,pyoscx.Rule.lessThan,entity=targetname)
    trig_cond1 = pyoscx.TraveledDistanceCondition(200)
    act_stop_trigger = pyoscx.EntityTrigger('Act_StopTrigger',0, pyoscx.ConditionEdge.none, trig_cond1,'hero', pyoscx.TriggeringEntitiesRule.any, 'stop')
    #andtrigger.add_condition(collision_trigger)
    #act_stoptrigger = pyoscx.ValueTrigger('stoptrigger',0,pyoscx.ConditionEdge.rising,pyoscx.SimulationTimeCondition(0,pyoscx.Rule.greaterThan))
    act = pyoscx.Act('my_act', act_start_trigger, act_stop_trigger)
    story.add_act(act)

    ######  Storyboard-->Story-->Act-->Hero ManeuverGroup
    mangr = pyoscx.ManeuverGroup('hero_mangroup')
    mangr.add_actor('hero')
    man = pyoscx.Maneuver('hero_maneuver')
    mangr.add_maneuver(man)
    act.add_maneuver_group(mangr)

    #########  Storyboard-->Story-->Act-->Hero ManeuverGroup-->Hero routing event
    hero_routing_event = pyoscx.Event('Hero_RoutingEvent',pyoscx.Priority.overwrite)
    man.add_event(hero_routing_event)

    hero_route = pyoscx.Route('Hero_Route')
    hero_routing_action = pyoscx.AssingRouteAction(hero_route)
    hero_routing_event.add_action('Hero_RoutingAction', hero_routing_action)

    # Event only has StartTrigger
    trigcond = pyoscx.SimulationTimeCondition(0.2,pyoscx.Rule.greaterThan)
    hero_routing_event_trigger = pyoscx.ValueTrigger('herotrigger',0.1,pyoscx.ConditionEdge.rising,trigcond,'start')
    hero_routing_event.add_trigger(hero_routing_event_trigger)
    #stoptrigcond = pyoscx.ReachPositionCondition(pyoscx.WorldPosition(round(hero_pos_list[-1][0],2),round(hero_pos_list[-1][1],2),round(hero_pos_list[-1][2]),2),0.2)
    #stoptrigger = pyoscx.EntityTrigger('herostoptrigger',0.2,pyoscx.ConditionEdge.rising,stoptrigcond,'hero',pyoscx.TriggeringEntitiesRule.any,'stop')
    #event.add_trigger(stoptrigger)

    # Storyboard-->Story-->Act-->Hero event-->route action
    positionlist = []
    for j in range(1,len(hero_pos_list)):
        x=round(hero_pos_list[j][0], 2)
        y=round(hero_pos_list[j][1], 2)
        z=round(hero_pos_list[j][2], 2)
        positionlist.append(pyoscx.WorldPosition(x,y,z))
        hero_route.add_waypoint(pyoscx.WorldPosition(x, y, z), pyoscx.RouteStrategy.fastest)

    #########  Storyboard-->Story-->Act-->Hero ManeuverGroup-->Hero Stop event
    hero_stop_event = pyoscx.Event('Hero_StopEvent',pyoscx.Priority.overwrite)
    man.add_event(hero_stop_event)
    step_time = pyoscx.TransitionDynamics(pyoscx.DynamicsShapes.step,pyoscx.DynamicsDimension.time,1)
    hero_stop_action = pyoscx.AbsoluteSpeedAction(0,step_time)
    hero_stop_event.add_action('Hero_StopAction', hero_stop_action)

    hero_ap_action = pyoscx.ActivateControllerAction(lateral=True, longitudinal=True)
    hero_stop_event.add_action('Hero_AutopilotAction', hero_ap_action)


    trigcond = pyoscx.ReachPositionCondition(pyoscx.WorldPosition(round(hero_pos_list[-1][0],2),round(hero_pos_list[-1][1],2),round(hero_pos_list[-1][2]),2),3)
    trigger = pyoscx.EntityTrigger('HeroStopTrigger', 0.2, pyoscx.ConditionEdge.rising, trigcond, 'hero', pyoscx.TriggeringEntitiesRule.any, 'start')
    hero_stop_event.add_trigger(trigger)

    #########  Storyboard-->Story-->Act-->Hero ManeuverGroup-->Hero Autopilot event





    """
    ## Reserved for FollowTrajectoryAction mode
    hero_timeline=np.linspace(0,5,len(hero_pos_list)-1)
    polyline = pyoscx.Polyline(hero_timeline,positionlist)
    #traj = pyoscx.Trajectory('hero_trajectory','False')
    traj = pyoscx.Trajectory('hero_trajectory',False)
    traj.add_shape(polyline)
    trajact = pyoscx.FollowTrajectoryAction(traj,pyoscx.FollowMode.position,pyoscx.ReferenceContext.relative,1,0)
    hero_event.add_action('hero_follow_trajectory',trajact)
    """

    #########  Storyboard-->Story-->Act-->ManeuverGroup-->NPCs event
    for i in range(len(actor_name_list)):
        actor_name='actor'+str(i)

        if len(actor_pos_list[i])>2:
            ######  Storyboard-->Story-->Act-->NPC ManeuverGroup
            man = pyoscx.Maneuver(actor_name+'_maneuver')
            mangr = pyoscx.ManeuverGroup(actor_name+'_mangroup')
            mangr.add_actor(actor_name)
            mangr.add_maneuver(man)
            act.add_maneuver_group(mangr)

            ######  Storyboard-->Story-->Act-->NPC ManeuverGroup-->NPC routing event
            npc_routing_event = pyoscx.Event(actor_name+'_RoutingEvent',pyoscx.Priority.overwrite)
            man.add_event(npc_routing_event)

            actor_route = pyoscx.Route(actor_name+'_route')
            actor_routing_action = pyoscx.AssingRouteAction(actor_route)
            npc_routing_event.add_action(actor_name+'_RoutingAction',actor_routing_action)

            #add simulationTime StartTrigger for other actors' event
            #trigcond1 = pyoscx.SimulationTimeCondition(0.2,pyoscx.Rule.greaterThan)
            #trigger1 = pyoscx.ValueTrigger(actor_name+'_trigger',0.1,pyoscx.ConditionEdge.rising,trigcond,'start')
            #npc_event.add_trigger(trigger1)
            trigcond2 = pyoscx.TraveledDistanceCondition(1)
            trigger2 = pyoscx.EntityTrigger('start_trigger', 0, pyoscx.ConditionEdge.none, trigcond2, 'hero')
            npc_routing_event.add_trigger(trigger2)

            # Storyboard-->Story-->Act-->npc event-->route action
            positionlist = []
            for j in range(len(actor_pos_list[i])):
                x=round(actor_pos_list[i][j][0], 2)
                y=round(actor_pos_list[i][j][1], 2)
                z=round(actor_pos_list[i][j][2], 2)
                positionlist.append(pyoscx.WorldPosition(x,y,z))
                actor_route.add_waypoint(pyoscx.WorldPosition(x, y, z), pyoscx.RouteStrategy.fastest)

            #########  Storyboard-->Story-->Act-->NPC ManeuverGroup-->NPC Stop event
            npc_stop_event = pyoscx.Event(actor_name+'_StopEvent',pyoscx.Priority.overwrite)
            man.add_event(npc_stop_event)
            step_time = pyoscx.TransitionDynamics(pyoscx.DynamicsShapes.step,pyoscx.DynamicsDimension.time,1)
            npc_stop_action = pyoscx.AbsoluteSpeedAction(0,step_time)
            npc_stop_event.add_action(actor_name+'_StopAction', npc_stop_action)
            
            trigcond = pyoscx.ReachPositionCondition(pyoscx.WorldPosition(round(actor_pos_list[i][-1][0],2),round(actor_pos_list[i][-1][1],2),round(actor_pos_list[i][-1][2]),2),3)
            trigger = pyoscx.EntityTrigger(actor_name+'_StopTrigger', 0.2, pyoscx.ConditionEdge.rising, trigcond, actor_name, pyoscx.TriggeringEntitiesRule.any, 'start')
            npc_stop_event.add_trigger(trigger)


            print(2)
        else:
            print(1)


    #====================================
    ####  Storyboard-->StopTrigger
    #====================================
    #sb_trigger1 = pyoscx.ValueTrigger('stop_simulation',0,pyoscx.ConditionEdge.rising, pyoscx.SimulationTimeCondition(10,pyoscx.Rule.greaterThan),'stop')
    sb_trigger1 = pyoscx.ValueTrigger('criteria_RunningStopTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('','', pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger1)
    sb_trigger2 = pyoscx.ValueTrigger('criteria_RunningRedLightTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('','', pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger2)
    sb_trigger3 = pyoscx.ValueTrigger('criteria_WrongLaneTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('','', pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger3)
    sb_trigger4 = pyoscx.ValueTrigger('criteria_OnSidewalkTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('','', pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger4)
    sb_trigger5 = pyoscx.ValueTrigger('criteria_KeepLaneTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('','', pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger5)
    sb_trigger6 = pyoscx.ValueTrigger('criteria_CollisionTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('','', pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger6)
    sb_trigger7 = pyoscx.ValueTrigger('criteria_DrivenDistanceTest',0,pyoscx.ConditionEdge.rising, pyoscx.ParameterCondition('distance_success',100, pyoscx.Rule.lessThan), 'stop')
    sb_stoptrigger.add_condition(sb_trigger7)

#########################################
####  Scenario
#########################################
    #sce = pyoscx.Scenario('CARLA:trajectory_example','Critical Scenario Generation', paramdec, entities=entities, storyboard = sb, roadnetwork=road, catalog=catalog)
    sce = pyoscx.Scenario('trajectory_example','Critical Scenario Generation', paramdec, entities=entities, storyboard = sb, roadnetwork=road, catalog=catalog)
    # display the scenario
    #pyoscx.prettyprint(sce.get_element())

    # if you want to save it
    #output_filename='test.xosc'                                   
    sce.write_xml(output_filename,True)
    #replace(output_filename, 'decription', 'description')


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
def main():
    argparser = argparse.ArgumentParser(description='OpenSCENARIO spec generation')

    argparser.add_argument('--input_xml', default='', help='input xml spec file') 
    argparser.add_argument('--map_name', default='', help='input xodr file') 
    argparser.add_argument('--output_filename', default='', help='output OpenSCENARIO file') 
    argparser.add_argument('--controller', default='', help='control mdule. npc or simple (simple control)') 
    argparser.add_argument('--case', default='', help='case name , used to simplify the commnad line input') 
    args = argparser.parse_args()

    casename=args.case
    input_xml=casename+'.xml'
    if args.map_name:
        map_name=args.map_name
    else:
        map_name=casename+'_map.xodr'
    output_filename=casename+'_'+args.controller+'.xosc'

    #generateScenario(args.input_xml, args.map_name, args.output_filename, args.controller)
    generateScenario(input_xml, map_name, output_filename, args.controller)
    print("Converting xml spec %s and map %s to OpenScenario spec %s Done!!!"%(args.input_xml, args.map_name, args.output_filename))


if __name__ == "__main__":
    main()


