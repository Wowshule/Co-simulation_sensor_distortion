import carla
import random
# import time
import argparse

# Connect to the client and retrieve the world object

def parse_args():
    parser = argparse.ArgumentParser(description="Mian camera location transform")
    parser.add_argument('-x', type=float, default=-7, help='Input argument x to move horizon (float type)')
    parser.add_argument('-y', type=float, default=0, help='Input argument y to move horizon (float type)')
    parser.add_argument('-z', type=float, default=7, help='Input argument z to move vertical (float type)')
    parser.add_argument('-p', type=float, default=-25, help='Input argument pitch (float type)')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    x = args.x
    z = args.z
    y = args.y
    p = args.p


    client = carla.Client('localhost', 2000)
    # client.reload_world()
    world = client.get_world()
    print(client.get_available_maps())

    ego_car = world.get_actors().filter('*vehicle.tesla.model3')
    id = ego_car[0].id
    spectator = world.get_spectator()


    while True:

        world_snapshot = world.wait_for_tick()
        actor_snapshot = world_snapshot.find(id)
        ego_car_location = actor_snapshot.get_transform()
        trans_position = ego_car_location.transform(carla.Location(x,y,z))

        
        ego_car_location.location.x = trans_position.x
        ego_car_location.location.y = trans_position.y
        ego_car_location.location.z = trans_position.z
        ego_car_location.rotation.pitch = p

        # carla.Transform(carla.Location(x=2.5,z=0.7))
        spectator.set_transform(ego_car_location)
        
        # time.sleep(0.3)