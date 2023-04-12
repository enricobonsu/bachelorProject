import carla


# --------------
# Spawn ego vehicle
# --------------
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(60.0)
world = client.get_world()
map = world.get_map()
actor_list = world.get_actors()
vehicle = actor_list.filter('vehicle.*')[0]
steps = 0
myData = []
tempData = []
while True:

    world.wait_for_tick()
    # print()
    blueprint =actor_list.filter('sensor.other.lane_invasion')[0]
    # print(blueprint)
    blueprint.listen(lambda x: print(x))
    # x = world.get_blueprint_library().find
    # print(actor_list)
    # x = carla.LaneInvasionEvent(actor=)
    ## current center of the vehicle.
    # cur = vehicle.get_location()
    # ##
    # trans = map.get_waypoint(vehicle.get_location()).transform

    # ## current road center x,y
    # cent = trans.location 
    # ## Gets lane width
    # roadWidth = map.get_waypoint(vehicle.get_location()).lane_width  
    # ## Euclidean distance between center road and center vehicle.
    # offsetCenter = 0
    # # if abs(control.steer) > 0.0:
    # #     print("offcenter = " + "{:.2f}".format(offsetCenter) +". road rotation = " , (round(trans.rotation.yaw,0) % 90) , ". vehicle rotation = ",(round(self._vehicle.get_transform().rotation.yaw,0) % 90), ". Steering angle = " + "{:.2f}".format(control.steer))
    # # else:
    # #     print("offcenter = " + "{:.2f}".format(offsetCenter) +". road rotation = " , (round(trans.rotation.yaw,0) % 90) , ". vehicle rotation = ", (round(self._vehicle.get_transform().rotation.yaw,0) % 90))
    # # The difference between the rotation of the vehicle and the center of the road.
    # diffCenter = cent-cur
    # # Is right of the center
    # # Negative offset from center means left of the center.
    # # Positive means right of center.
    # # if (offsetCenter >= 0.01):
    # if (abs(diffCenter.x) > abs(diffCenter.y)):
    #     if diffCenter.x > 0:
    #         offsetCenter = -diffCenter.x
    #     else:
    #         offsetCenter = diffCenter.x
    # else: # y-cor determines placing of the road.
    #     if diffCenter.y > 0:
    #         offsetCenter = -diffCenter.y
    #     else:
    #         offsetCenter = diffCenter.y
    
    # differenceInForward = carla.Vector3D.distance_2d(vehicle.get_transform().get_forward_vector(),trans.get_forward_vector())
    # if abs(vehicle.get_control().steer) > 0.0:
    #     print("offcenter = " + "{:.3f}".format(offsetCenter) +". diff in rotation = ",diffCenter, round(differenceInForward,2), ". Steering angle = " + "{:.2f}".format(vehicle.get_control().steer))
    # else:
    #     print( "offcenter = " + "{:.3f}".format(offsetCenter) +". diff in rotation = " , round(differenceInForward,2))
    # tempData.append(["{:.3f}".format(offsetCenter)])
