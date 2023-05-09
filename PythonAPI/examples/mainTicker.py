import carla
import time
import trafficLoop
import argparse
from script import RecordData


def setup():
    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    sim_world = client.get_world()

    settings = sim_world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.113975
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = 16
    sim_world.apply_settings(settings)
    # client.load_world(map_name="Town10HD_Opt",
    #                   reset_settings=False, map_layers=carla.MapLayer.NONE)
    # print(settings.substepping)

    traffic_manager = client.get_trafficmanager()
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


def tick():
    world = None

    try:
        test = args()
        (client, traffic_manager) = setup()

        (hud, world) = trafficLoop.game_loop(test, client, traffic_manager)

        client.get_world().tick()
        world.setup_sensors()
        record = RecordData(client)
        # world.player.get_velocity()

        (world, agent, destination) = trafficLoop.game_loop2(
            traffic_manager, world)
        # print()
        # agent._vehicle.set_target_velocity(30.0)
        # Change Vehicle Physics Control parameters of the vehicle
        # front_left_wheel  = carla.WheelPhysicsControl(tire_friction=0.0, damping_rate=1.0, max_steer_angle=70.0, radius=30.0)
        # front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.0, damping_rate=1.5, max_steer_angle=70.0, radius=25.0)
        # rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=0.0, damping_rate=0.2, max_steer_angle=0.0,  radius=15.0)
        # rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=0.0, damping_rate=1.3, max_steer_angle=0.0,  radius=20.0)

        # wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
        physics_control = agent._vehicle.get_physics_control()

        # print(physics_control.wheels[0].tire_friction)
        wheels = []
        for wheel in physics_control.wheels:
            wheelToAdd = carla.WheelPhysicsControl(
                tire_friction=0.0, damping_rate=wheel.damping_rate, max_steer_angle=wheel.max_steer_angle, radius=wheel.radius,
                max_brake_torque=wheel.max_brake_torque, max_handbrake_torque=wheel.max_handbrake_torque, position=wheel.position,
                long_stiff_value=wheel.long_stiff_value, lat_stiff_max_load=wheel.lat_stiff_max_load, lat_stiff_value=wheel.lat_stiff_value)
            wheels.append(wheelToAdd)

        physics_control.wheels = wheels
        agent._vehicle.apply_physics_control(physics_control)
        # physics_control = agent._vehicle.get_physics_control()
        # print(physics_control.wheels[0].tire_friction)
        # physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
        # physics_control.max_rpm = 10000
        # physics_control.moi = 1.0
        # physics_control.damping_rate_full_throttle = 0.0
        # physics_control.use_gear_autobox = True
        # physics_control.gear_switch_time = 0.5
        # physics_control.clutch_strength = 10
        # physics_control.mass = 10000
        # physics_control.drag_coefficient = 0.25
        # physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
        # physics_control.wheels = wheels

        # Apply Vehicle Physics Control for the vehicle

        while True:
            # clock.tick()
            client.get_world().tick()
            print(record.recordPassingLightDemonstration(goal_loc=destination))
            # record.drawTrafficLightTriggers()
            trafficLoop.tick_action(world,
                                    agent, destination, traffic_manager)
    finally:
        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

            world.destroy()

        # pygame.quit()
    # print(record.recordPassingLightDemonstration())


if __name__ == '__main__':
    tick()
