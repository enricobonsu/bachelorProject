import carla
import math
import numpy as np
import pandas as pd


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
compassVal = 0
currentTrajCrossed = False
finalData = []
trajCounter = 0
fileCounter = 11

# Updates if current trajectory has crossed a lane.


def currentTrajUpdate():
    global currentTrajCrossed
    currentTrajCrossed = True


def calcCompass(sensor_data):
    global compassVal
    compassVal = int(math.degrees(sensor_data.compass))
    # print(compassVal)


crossingLaneSensorActive = False  # is true when crossing lane sensor is active

# Stores the start time when a trajectory is being recorded.
# py -3.7 config.py --delta-seconds 0.01 # set tick timer on 0.1 second.

while True:
    current_waypoint = map.get_waypoint(vehicle.get_location())
    snapshot = world.wait_for_tick()

    if (not crossingLaneSensorActive):
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        sensor.listen(lambda event: currentTrajUpdate())
        bp2 = world.get_blueprint_library().find('sensor.other.imu')
        sensor2 = world.spawn_actor(
            bp2, carla.Transform(), attach_to=vehicle)
        sensor2.listen(
            lambda sensor_data: calcCompass(sensor_data))
        crossingLaneSensorActive = True

    # print(compassVal)

    # current center of the vehicle.
    cur = vehicle.get_location()
    veh = vehicle.get_transform()
    ##
    wp = map.get_waypoint(vehicle.get_location())
    trans = map.get_waypoint(vehicle.get_location()).transform
    # begin = carla.Location(trans.location.x, trans.location.y, 0)
    # end = carla.Location(trans.location.x, trans.location.y, 50)
    # debugger = world.debug
    # debugger.draw_line(begin, end, thickness=0.1, life_time=1)
    # current road center x,y
    cent = trans.location

    # Gets lane width
    # roadWidth = map.get_waypoint(vehicle.get_location()).lane_width

    # Euclidean distance between center road and center vehicle.
    offsetCenter = 0

    # The difference between the center of the vehicle and the center of the road.
    diffCenter = cur-cent
    # Is right of the center
    # Negative offset from center means left of the center.
    # Positive means right of center.
    diffForward = trans.rotation

    # both are negative,
    if (abs(diffCenter.x) > abs(diffCenter.y)):
        if (135 < compassVal < 225):
            if diffCenter.x > 0:
                offsetCenter = -diffCenter.x
                # print("x = left", offsetCenter, compassVal)
            else:
                offsetCenter = -diffCenter.x
                # print("x = right", offsetCenter, compassVal)
        else:
            if diffCenter.x > 0:
                offsetCenter = diffCenter.x
                # print("x = right", offsetCenter, compassVal)
            else:
                offsetCenter = diffCenter.x
                # print("x = left", offsetCenter, compassVal)

    else:  # y-cor determines placing of the road.
        if (0 < compassVal < 180):
            if diffCenter.y > 0:
                offsetCenter = diffCenter.y
                # print("y = right", offsetCenter, compassVal)
            else:
                offsetCenter = diffCenter.y
                # print("y = left", offsetCenter, compassVal)
        else:
            if diffCenter.y > 0:
                offsetCenter = -diffCenter.y
                # print("y = left", offsetCenter, compassVal)
            else:
                offsetCenter = -diffCenter.y
                # print("y = right", offsetCenter, compassVal)

    # # # Calculates difference in rotation of road and vehicle
    differenceInForward = carla.Vector3D.distance_2d(
        vehicle.get_transform().get_forward_vector(), trans.get_forward_vector())

    # print(snapshot.timestamp.delta_seconds)
    # # # print()
    print(snapshot.timestamp.elapsed_seconds)
    if (not wp.is_junction):
        if abs(vehicle.get_control().steer) > 0.0:
            # +" diff rot {:.1f} ".format((veh.rotation.yaw -trans.rotation.yaw))
            # "delta: ", snapshot.timestamp.delta_seconds,
            # "current_waypoint: ", current_waypoint.id,  zegt niets, alleen dat er heel veel de wegen zijn opgedeeld in heel veel stukken
            print("is junction " + str(wp.is_junction), "road rot = {:.1f}".format(trans.rotation.yaw), "veh rot = {:.1f}".format(veh.rotation.yaw),
                  "offcenter =", round(offsetCenter, 3), "steering = ",  round(vehicle.get_control().steer, 2))
            # print("offcenter = " + "{:.3f}".format(
            #     offsetCenter) + ". diff in rotation = " +  "{:.5f}".format(differenceInForward) + ". Steering angle = " + "{:.2f}".format(vehicle.get_control().steer))
        else:
            print("is junction " + str(wp.is_junction), "road rot = {:.1f}".format(trans.rotation.yaw), "veh rot = {:.1f}".format(veh.rotation.yaw), "offcenter =", round(offsetCenter, 3))
    # print("offcenter = " +
    #       "{:.3f}".format(offsetCenter) + ". diff in rotation = ","{:.5f}".format(differenceInForward))

    # terminal state is wanneer deze wp.is_junction true is
    # timestamp elapsed_seconds == tijd in simulator.
    if (len(tempData) <  100):
        tempData.append([round(offsetCenter, 3), round(
            differenceInForward, 4), round(vehicle.get_control().steer, 2)])
    else:
        # Check if no lanes are crossed
        if (not currentTrajCrossed):
            finalData.append(tempData.copy())
            trajCounter += 1
        else:
            currentTrajCrossed = False
        tempData.clear()
        if (len(finalData) == 10):

            offsets = []
            differenceForward = []
            steeringAngle = []
            for trajectory in finalData:
                offsets = offsets + [item[0] for item in trajectory]
                differenceForward = differenceForward + \
                    [item[1] for item in trajectory]
                steeringAngle = steeringAngle + \
                    [item[2] for item in trajectory]

            pd.DataFrame({"offsetCenter": offsets, "vehicle rotation": veh.rotation.yaw,"road rotation": trans.rotation.yaw,
                         "steeringAngle": steeringAngle, "is_junction":wp.is_junction}).to_csv('trajectories'+str(fileCounter)+'.csv', index=False)
            exit()
        # exit()
