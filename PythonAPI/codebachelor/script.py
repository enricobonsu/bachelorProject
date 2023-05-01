import carla
import math
import numpy as np
import pandas as pd
import misc


class RecordData(object):

    def __init__(self):
        # --------------
        # Spawn ego vehicle
        # --------------
        self.client = carla.Client('127.0.0.1', 2000)
        self.client.set_timeout(60.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.actor_list = self.world.get_actors()
        self.vehicle = self.actor_list.filter('vehicle.*')[0]
        self.lights_list = self.actor_list.filter("*traffic_light*")
        self.bbs = self.world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
        self.debugger = self.world.debug
        self._last_traffic_light = None

        # Dictionary mapping a traffic light to a wp corrspoing to its trigger volume location
        self._lights_map = {}

    # Draws waypoints that are the triggers for the traffic lights.

    def drawTrafficLightTriggers(self, seconds=1):
        for light in self.lights_list:
            for point in light.get_affected_lane_waypoints():
                begin = carla.Location(
                    point.transform.location.x, point.transform.location.y, 0)
                end = carla.Location(point.transform.location.x,
                                     point.transform.location.y, 50)
                # life_time = 0 is permanent.
                self.debugger.draw_line(begin, end, thickness=0.1,
                                        life_time=seconds, color=carla.Color(0, 255, 0, 0))

    #  Draws forward vector line of the vehicle.
    def forwardVectorLine(self, seconds=1):
        loc = self.vehicle.get_location()
        begin = carla.Location(loc.x, loc.y, loc.z +
                               self.vehicle.bounding_box.extent.z*2)
        end = carla.Location(loc.x + self.vehicle.get_transform().get_forward_vector().x * 3, loc.y +
                             self.vehicle.get_transform().get_forward_vector().y * 3, loc.z + self.vehicle.bounding_box.extent.z * 2)
        # life_time = 0 is permanent.
        self.debugger.draw_line(begin, end, thickness=0.1, life_time=seconds)

    def _affected_by_traffic_light(self, lights_list=None, max_distance=0.0):
        """
        Method to check if there is a red light affecting the vehicle.

            :param lights_list (list of carla.TrafficLight): list containing TrafficLight objects.
                If None, all traffic lights in the scene are used
            :param max_distance (float): max distance for traffic lights to be considered relevant.
                If None, the base threshold value is used
        """
        if not lights_list:
            lights_list = self.world.get_actors().filter("*traffic_light*")

        if self._last_traffic_light:
            if self._last_traffic_light.state != carla.TrafficLightState.Red:
                self._last_traffic_light = None
            else:
                return (1, self._last_traffic_light)

        ego_vehicle_location = self.vehicle.get_location()
        ego_vehicle_waypoint = self.map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            if traffic_light.id in self._lights_map:
                trigger_wp = self._lights_map[traffic_light.id]
            else:
                trigger_location = misc.get_trafficlight_trigger_location(
                    traffic_light)
                trigger_wp = self.map.get_waypoint(trigger_location)
                self._lights_map[traffic_light.id] = trigger_wp

            if trigger_wp.transform.location.distance(ego_vehicle_location) > max_distance:
                continue
            # print("distance",trigger_wp.transform.location.distance(ego_vehicle_location))
            # print("trigger_wp.road_id", trigger_wp.road_id,
            #       "ego_vehicle_waypoint.road_id", ego_vehicle_waypoint.road_id)

            if trigger_wp.road_id != ego_vehicle_waypoint.road_id:
                continue
            # print("distance",trigger_wp.transform.location.distance(ego_vehicle_location))
            ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
            wp_dir = trigger_wp.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z
            # print("TEST")

            if dot_ve_wp < 0:
                return (2, None)

            if traffic_light.state != carla.TrafficLightState.Red:
                if misc.is_within_distance(trigger_wp.transform, self.vehicle.get_transform(), max_distance, [0, 90]):
                    # print("light not red, but close to")
                    return (3, None)
                continue
           #  dot negative == car is between 90 and 270 degrees from the light trigger wp, thus faced backwards.
            if misc.is_within_distance(trigger_wp.transform, self.vehicle.get_transform(), max_distance, [0, 90]):
                self._last_traffic_light = traffic_light
                # print("Light is red")
                return (1, traffic_light)

        return (0, None)

    def recordPassingLightDemonstration(self, trajCount=10, start_loc=None, goal_loc=None):
        demo = []
        traj = []
        start = self.map.get_waypoint(
            location=carla.Location(x=-45.0, y=78.0, z=0.0)).transform
        start.location.z += 0.1
        end = self.map.get_waypoint(
            location=carla.Location(x=-45.0, y=-30.0, z=0.0))
        while True:
            self.world.wait_for_tick()
            returnCode, light = self._affected_by_traffic_light(
                max_distance=2.0 + 0.3 * 30)
            # print()
            isInDistance = 0
            isRedLight = 0
            passedIntersection = 0
            stop = 0
            if (returnCode == 0):
                isInDistance = 0
                isRedLight = 0
                passedIntersection = 0
                stop = 1 if self.vehicle.get_velocity().length() < 1.0 else 0
            elif (returnCode == 1):
                isInDistance = 1
                isRedLight = 1
                passedIntersection = 0
                stop = 1 if self.vehicle.get_velocity().length() < 1.0 else 0
            elif (returnCode == 2):
                isInDistance = 0
                isRedLight = 0
                passedIntersection = 1
                stop = 1 if self.vehicle.get_velocity().length() < 1.0 else 0
            elif (returnCode == 3):
                isInDistance = 1
                isRedLight = 0
                passedIntersection = 0
                stop = 1 if self.vehicle.get_velocity().length() < 1.0 else 0

            traj.append([isInDistance, isRedLight, passedIntersection, stop])

            if passedIntersection == 1:
                if (len(traj) < 300):
                    traj.clear()
                    passedIntersection = 0 
                    continue
                else:
                    
                    demo.append(traj.copy())
                    print("traj added")
                    traj.clear()
                    # while not misc.is_within_distance(start, self.vehicle.get_transform(), 2, [0, 90]):
                    #     self.world.wait_for_tick()
                    #     print("clearing")
                    
                    # Misschien moet er gewacht worden tot we weer zijn terug geteleporteerd.
                    
                    if (len(demo) == 30):
                        isInDistances = []
                        isRedLights = []
                        passedIntersections = []
                        stops = []
                        for count, trajectory in enumerate(demo):
                            isInDistances = [item[0] for item in trajectory]
                            isRedLights = [item[1] for item in trajectory]
                            passedIntersections = [item[2] for item in trajectory]
                            stops = [item[3] for item in trajectory]

                            pd.DataFrame({"isInDistance": isInDistances, "isRedLight": isRedLights, "passedIntersection": passedIntersections,
                                    "stop": stops}).to_csv('traj'+str(count) + '.csv', index=False)
                    else:
                        print("len(demo)", len(demo))
            


# while True:
#     world.wait_for_tick()
#     # loc = vehicle.get_location()
#     # print(loc)
#     print(vehicle.get_transform())


# carla.map.get_waypoint()
# x = -45, y = 78 , z= 0 , rot(0,-90,0)
# x = -85, y = 24, z =0 , rot (0,0,0)
# while True:
#     for light in lights_list:
#         for point in light.get_affected_lane_waypoints():
#             begin = carla.Location(vehicle.get_transform().location.x, point.transform.location.y, 0)
#             end = carla.Location(point.transform.location.x, point.transform.location.y, 50)
#             debugger.draw_line(begin, end, thickness=0.1, life_time=1) # life_time = 0 is permanent.
# max_tlight_distance = 2.0 + 0.3 * vehicle_speed

    # loc = vehicle.get_location()
    # print("junction ",wp.is_junction, "road_id", wp.road_id, "lane_id", wp.lane_id, "section_id",wp.section_id, "lane_type", wp.lane_type, "is_at_traffic_light", vehicle.is_at_traffic_light() )


def main():
    record = RecordData()
    record.recordPassingLightDemonstration()
    # while True:
    #     record.world.wait_for_tick()
    #     record.drawTrafficLightTriggers()
    #     # wp = record.map.get_waypoint(record.vehicle.get_location())
    #     # print(wp)
    #     record._affected_by_traffic_light(max_distance=2.0 + 0.3 * 30)


if __name__ == '__main__':
    main()
# steps = 0
# myData = []
# tempData = []
# compassVal = 0
# currentTrajCrossed = False
# finalData = []
# trajCounter = 0
# fileCounter = 11

# # Updates if current trajectory has crossed a lane.


# def currentTrajUpdate():
#     global currentTrajCrossed
#     currentTrajCrossed = True


# def calcCompass(sensor_data):
#     global compassVal
#     compassVal = int(math.degrees(sensor_data.compass))
    # print(compassVal)


# crossingLaneSensorActive = False  # is true when crossing lane sensor is active

# # Stores the start time when a trajectory is being recorded.
# # py -3.7 config.py --delta-seconds 0.01 # set tick timer on 0.1 second.

# while True:
#     current_waypoint = map.get_waypoint(vehicle.get_location())
#     snapshot = world.wait_for_tick()

#     if (not crossingLaneSensorActive):
#         bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
#         sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
#         sensor.listen(lambda event: currentTrajUpdate())
#         bp2 = world.get_blueprint_library().find('sensor.other.imu')
#         sensor2 = world.spawn_actor(
#             bp2, carla.Transform(), attach_to=vehicle)
#         sensor2.listen(
#             lambda sensor_data: calcCompass(sensor_data))
#         crossingLaneSensorActive = True

#     # print(compassVal)

#     # current center of the vehicle.
#     cur = vehicle.get_location()
#     veh = vehicle.get_transform()
#     ##
#     wp = map.get_waypoint(vehicle.get_location())
#     trans = map.get_waypoint(vehicle.get_location()).transform
#     # begin = carla.Location(trans.location.x, trans.location.y, 0)
#     # end = carla.Location(trans.location.x, trans.location.y, 50)
#     # debugger = world.debug
#     # debugger.draw_line(begin, end, thickness=0.1, life_time=1)
#     # current road center x,y
#     cent = trans.location

#     # Gets lane width
#     # roadWidth = map.get_waypoint(vehicle.get_location()).lane_width

#     # Euclidean distance between center road and center vehicle.
#     offsetCenter = 0

#     # The difference between the center of the vehicle and the center of the road.
#     diffCenter = cur-cent
#     # Is right of the center
#     # Negative offset from center means left of the center.
#     # Positive means right of center.
#     diffForward = trans.rotation

#     # both are negative,
#     if (abs(diffCenter.x) > abs(diffCenter.y)):
#         if (135 < compassVal < 225):
#             if diffCenter.x > 0:
#                 offsetCenter = -diffCenter.x
#                 # print("x = left", offsetCenter, compassVal)
#             else:
#                 offsetCenter = -diffCenter.x
#                 # print("x = right", offsetCenter, compassVal)
#         else:
#             if diffCenter.x > 0:
#                 offsetCenter = diffCenter.x
#                 # print("x = right", offsetCenter, compassVal)
#             else:
#                 offsetCenter = diffCenter.x
#                 # print("x = left", offsetCenter, compassVal)

#     else:  # y-cor determines placing of the road.
#         if (0 < compassVal < 180):
#             if diffCenter.y > 0:
#                 offsetCenter = diffCenter.y
#                 # print("y = right", offsetCenter, compassVal)
#             else:
#                 offsetCenter = diffCenter.y
#                 # print("y = left", offsetCenter, compassVal)
#         else:
#             if diffCenter.y > 0:
#                 offsetCenter = -diffCenter.y
#                 # print("y = left", offsetCenter, compassVal)
#             else:
#                 offsetCenter = -diffCenter.y
#                 # print("y = right", offsetCenter, compassVal)

#     # # # Calculates difference in rotation of road and vehicle
#     differenceInForward = carla.Vector3D.distance_2d(
#         vehicle.get_transform().get_forward_vector(), trans.get_forward_vector())

#     # print(snapshot.timestamp.delta_seconds)
#     # # # print()
#     print(snapshot.timestamp.elapsed_seconds)
#     if (not wp.is_junction):
#         if abs(vehicle.get_control().steer) > 0.0:
#             # +" diff rot {:.1f} ".format((veh.rotation.yaw -trans.rotation.yaw))
#             # "delta: ", snapshot.timestamp.delta_seconds,
#             # "current_waypoint: ", current_waypoint.id,  zegt niets, alleen dat er heel veel de wegen zijn opgedeeld in heel veel stukken
#             print("is junction " + str(wp.is_junction), "road rot = {:.1f}".format(trans.rotation.yaw), "veh rot = {:.1f}".format(veh.rotation.yaw),
#                   "offcenter =", round(offsetCenter, 3), "steering = ",  round(vehicle.get_control().steer, 2))
#             # print("offcenter = " + "{:.3f}".format(
#             #     offsetCenter) + ". diff in rotation = " +  "{:.5f}".format(differenceInForward) + ". Steering angle = " + "{:.2f}".format(vehicle.get_control().steer))
#         else:
#             print("is junction " + str(wp.is_junction), "road rot = {:.1f}".format(trans.rotation.yaw), "veh rot = {:.1f}".format(veh.rotation.yaw), "offcenter =", round(offsetCenter, 3))
#     # print("offcenter = " +
#     #       "{:.3f}".format(offsetCenter) + ". diff in rotation = ","{:.5f}".format(differenceInForward))

#     # terminal state is wanneer deze wp.is_junction true is
#     # timestamp elapsed_seconds == tijd in simulator.
#     if (len(tempData) <  100):
#         tempData.append([round(offsetCenter, 3), round(
#             differenceInForward, 4), round(vehicle.get_control().steer, 2)])
#     else:
#         # Check if no lanes are crossed
#         if (not currentTrajCrossed):
#             finalData.append(tempData.copy())
#             trajCounter += 1
#         else:
#             currentTrajCrossed = False
#         tempData.clear()
#         if (len(finalData) == 10):

#             offsets = []
#             differenceForward = []
#             steeringAngle = []
#             for trajectory in finalData:
#                 offsets = offsets + [item[0] for item in trajectory]
#                 differenceForward = differenceForward + \
#                     [item[1] for item in trajectory]
#                 steeringAngle = steeringAngle + \
#                     [item[2] for item in trajectory]

#             pd.DataFrame({"offsetCenter": offsets, "vehicle rotation": veh.rotation.yaw,"road rotation": trans.rotation.yaw,
#                          "steeringAngle": steeringAngle, "is_junction":wp.is_junction}).to_csv('trajectories'+str(fileCounter)+'.csv', index=False)
#             exit()
#         # exit()
