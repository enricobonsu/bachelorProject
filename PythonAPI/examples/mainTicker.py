import carla
import time
import trafficLoop
import argparse
from script import RecordData
import pandas as pd


def setup():
    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    sim_world = client.get_world()

    settings = sim_world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.12
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = 16
    sim_world.apply_settings(settings)
    listOfBuildings = client.get_world().get_environment_objects(
        carla.CityObjectLabel.Buildings)
    if listOfBuildings:
        print("in if list")
        client.load_world(map_name="Town10HD_Opt",
                          reset_settings=False, map_layers=carla.MapLayer.NONE)
    # print(settings.substepping)

    traffic_manager = client.get_trafficmanager()
    # traffic_manager.auto_lane_change(False)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.global_percentage_speed_difference(0.0)
    return client, traffic_manager


def args():
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic", "Constant"],
        help="select which agent to run",
        default='Constant')
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)

    args = argparser.parse_args()
    args.sync = True  # Sync by default
    args.loop = True  # Loop by default
    args.width, args.height = [int(x) for x in args.res.split('x')]

    # log_level = logging.DEBUG if args.debug else logging.INFO
    # logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    # logging.info('listening to server %s:%s', args.host, args.port)

    # print(__doc__)
    return args


def main(scenario=0, case = 0 , drawObservationValues = False, drawStart = False, drawEnd = True):
    world = None

    try:
        test = args()
        (client, traffic_manager) = setup()
        (hud, world) = trafficLoop.game_loop(
            test, client, traffic_manager, scenario=scenario)

        client.get_world().tick()
        world.setup_sensors()
        record = RecordData(client)

        (world, agent, destination) = trafficLoop.game_loop2(
            traffic_manager, world, scenario=scenario)

        # Teleport spectator camera above the expert agents vehicle
        spectator = client.get_world().get_spectator()
        
        # Configures the agents vehicle.
        configVehicle(agent)

        features = None
        if drawEnd:
            client.get_world().debug.draw_point(
                location=destination.location, size=0.2, color=carla.Color(0, 255, 0), life_time=100.0)
        while True:
            if case == 1:
                transitions = dict()
                curStateSet = set()
                nextStateSet = set()
                stateFeatureList = list()

                for action in [True,False]:

                    currentTrans = agent._vehicle.get_transform()
                    featuresCurrent = record.recordPassingLightDemonstration(
                                goal_loc=destination, record=False)
                    stateFeatureList.append(featuresCurrent)
                    curStateSet.add(len(stateFeatureList)-1)
                    # if not nextStateSet:
                    #     # No previous state set (aka init state)
                    #     if featuresCurrent in stateFeatureList:
                    #         index = stateFeatureList.index(featuresCurrent)
                    #     else:
                    #         stateFeatureList.append(featuresCurrent)
                    #         index = len(stateFeatureList)-1

                    isDone = False

                    # Simulate 500 transitions
                    while not isDone:
                        for _ in range(500):
                            # client.get_world().tick()
                            agent._vehicle.set_transform(currentTrans)
                            
                            # Currently only two actions possible: stop or do not stop.
                            isDone = trafficLoop.tick_generate_transitions(world,
                                                        agent, destination, traffic_manager, brake = action)
                            client.get_world().tick()
                            featuresNew = record.recordPassingLightDemonstration(
                                    goal_loc=destination, record=False)
                            
                            # Check if states is known and add to the transition set
                            if featuresNew in stateFeatureList:
                                index = stateFeatureList.index(featuresNew)
                                nextStateSet.add(index)

                            else:
                                stateFeatureList.append(featuresNew)
                                index = len(stateFeatureList)-1
                                nextStateSet.add(index)
                            
                            print("From ",featuresCurrent, "To", featuresNew, "with action",action)
                        print("Performed 500 transitions")
                        if action:
                            print("Braking can loop between the followin states", nextStateSet)
                            for i in list(nextStateSet):
                                if str(i)+"-1" in transitions:
                                    transitions[str(i)+"-1"].update(nextStateSet)
                                else:
                                    transitions[str(i)+"-1"] = set(nextStateSet)
                            print(transitions)
                        else:
                            print("States", curStateSet, "can transition to states", nextStateSet)
                            for i in list(curStateSet):
                                if str(i)+"-0" in transitions:
                                    transitions[str(i)+"-0"].update(nextStateSet)
                                else:
                                    transitions[str(i)+"-0"] = set(nextStateSet)
                        curStateSet = set(nextStateSet)
                        nextStateSet.clear()
                        if action:
                            # We perform brakes, thus after a cycle of transitions we move 1 time
                            isDone = trafficLoop.tick_generate_transitions(world,
                                                        agent, destination, traffic_manager, brake = False)
                            client.get_world().tick()
                            trafficLoop.tick_generate_transitions(world,
                                                        agent, destination, traffic_manager, brake = True)
                            client.get_world().tick()
                            currentTrans = agent._vehicle.get_transform()
                            featuresCurrent = record.recordPassingLightDemonstration(
                                    goal_loc=destination, record=False)
                        else:
                            currentTrans = agent._vehicle.get_transform()
                            featuresCurrent = featuresNew
                    
                df = pd.DataFrame([range(len(stateFeatureList)),stateFeatureList]).transpose()
                df.columns = ['state', 'features']
                df.to_csv('stateFeatures.csv', index=False)
                print(df)
                print("\n\n\n")
                df2 = pd.DataFrame([list(transitions.keys()),list(transitions.values())]).transpose()
                df2.columns = ['state-action', 'transitions']
                df2.to_csv('stateTransitions.csv', index=False)
                print(df2)
                exit()
            if case == 2:
                trafficLoop.tick_irl_agent(world,agent, destination, traffic_manager)
                client.get_world().tick()
            else:
                # record.drawTrafficLightTriggers()

                # Will generate the observed feature values. Will also record this if record=True
                # features = record.recordPassingLightDemonstration(
                #                         goal_loc=destination, record=False)
                
                # attachSpectatorToAgent(spectator,agent)

                # client.get_world().debug.draw_point(
                #     location=carla.Location(x=5.0, y=-64.4, z=0.0), size=0.2, color=carla.Color(0, 0, 255))
                if drawObservationValues:
                    if features == None:
                        features = record.recordPassingLightDemonstration(
                            goal_loc=destination, record=False)
                    drawObservation(client, agent, features)
                trafficLoop.tick_action(world,
                                        agent, destination, traffic_manager, scenario=scenario)
    finally:
        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

            world.destroy()

    # print(record.recordPassingLightDemonstration())


def drawObservation(client, agent, features):
    veh_loc = agent._vehicle.get_location()
    print(features)

    # Draw observed feature values above the vehicle
    client.get_world().debug.draw_string(location=carla.Location(veh_loc.x-0.5, veh_loc.y, veh_loc.z + 1),
                                         text=str(features[0]) + " " + str(features[1]) + " " + str(features[2]) + " " + str(features[3]), color=carla.Color(0, 255, 0))


def attachSpectatorToAgent(spectator, agent):
    agent_transform = agent._vehicle.get_transform()
    agent_transform.location.z += 4
    spec_transform = spectator.get_transform()
    print(spec_transform)
    abs_vector = abs(agent_transform.rotation.get_forward_vector())
    print(abs_vector)
    changing_axis = 0
    if (abs_vector.y > abs_vector.x):
        changing_axis = 1

    if (changing_axis):
        spec_transform.location.y = agent_transform.location.y
    else:
        spec_transform.location.x = agent_transform.location.x
    # Get directation that the vehicle is moving
    # print(agent_transform.rotation.get_right_vector())
    print(agent_transform.location)
    print("\n\n\n")
    spectator.set_transform(spec_transform)


def configVehicle(agent):
    # Set frication of wheels to 1.0
    physics_control = agent._vehicle.get_physics_control()
    wheels = []
    for wheel in physics_control.wheels:
        wheelToAdd = carla.WheelPhysicsControl(
            tire_friction=0.0, damping_rate=wheel.damping_rate, max_steer_angle=wheel.max_steer_angle, radius=wheel.radius,
            max_brake_torque=wheel.max_brake_torque, max_handbrake_torque=wheel.max_handbrake_torque, position=wheel.position,
            long_stiff_value=wheel.long_stiff_value, lat_stiff_max_load=wheel.lat_stiff_max_load, lat_stiff_value=wheel.lat_stiff_value)
        wheels.append(wheelToAdd)

    physics_control.wheels = wheels
    agent._vehicle.apply_physics_control(physics_control)


if __name__ == '__main__':
    """
    CASE:
    * 0 = observe expert agent
    * 1 = Generate state transition function
    * 2 = Run learner agent (with reward function)
    """
    main(scenario=0, case = 1, drawObservationValues = False)
