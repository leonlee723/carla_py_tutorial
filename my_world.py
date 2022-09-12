'''
My world for Autoware.auto on Nvidia Jetson platform
'''
import carla
from carla import Transform, Rotation, Location
import random
import math
import numpy as np
from queue import Queue
from queue import Empty

def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def main():
    sensor_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        
        print("client version:" + client.get_client_version())
        print("server version:" + client.get_server_version())
        
        maps = client.get_available_maps()
        for map in maps:
            print("maps:"+map)
        
        world = client.load_world('Town05_opt')
        # settings = world.get_settings()
        # settings.fixed_delta_seconds = 0.05 #20 fps, 5ms
        # settings.synchronous_mode = True
        # world.apply_settings(settings)
        spectator = world.get_spectator()
        
        
        vehicle_blueprints = world.get_blueprint_library().filter('vehicle')
        for blueprint in vehicle_blueprints:
            print(blueprint.id)
        
        sensor_queue = Queue()
        
        # set weather
        
        # set model3 as ego
        blueprint_library = world.get_blueprint_library()
        # retrieve model3
        ego_bp = blueprint_library.find('vehicle.tesla.model3')
        # ego_location = random.choice(world.get_map().get_spawn_points()).location
        # ego_location.z += 2.0
        # ego_transform = carla.Transform(ego_location, carla.Rotation(roll = 0.0, pitch=0.0, yaw=0))
        ego_transform = random.choice(world.get_map().get_spawn_points())
        print(ego_transform)
        ego_transform.rotation.yaw = -90
        print(ego_transform)
        ego_location = ego_transform.location
        # ego_location.z += 2.0
        # spawn_point.location.z += 2.0
        # spawn_point.rotation.roll = 0.0
        # spawn_point.rotation.pitch = 0.0
        # spawn ego actor
        ego_actor = world.spawn_actor(ego_bp, ego_transform)
        
        spectator_location = carla.Location(0, 6, 2) + ego_location
        print(spectator_location)
        
        spectator.set_transform(carla.Transform(spectator_location, carla.Rotation(roll = 0.0, pitch=0.0, yaw=-90)))
        
        
        
    finally:
        for sensor in sensor_list:
            sensor.destroy()
        # ego_actor.destroy()
        print('done.')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user')