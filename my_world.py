'''
My world for Autoware.auto on Nvidia Jetson platform
'''
import carla
from carla import Transform, Rotation, Location
import random
import math
import os
import logging
import numpy as np
from queue import Queue
from queue import Empty

def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def sensor_callback(sensor_data, sensor_queue, sensor_name):
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join('./lidar', '%06d.ply' % sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join('./camera', '%06d.png' % sensor_data.frame))
    sensor_queue.put((sensor_data.frame, sensor_name))

def main():
    sensor_list = []
    actor_list = []
    vehicles_id_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        synchronous_master = True
        
        print("client version:" + client.get_client_version())
        print("server version:" + client.get_server_version())
        
        maps = client.get_available_maps()
        for map in maps:
            print("maps:"+map)
        
        world = client.load_world('Town01_opt')
        map = world.get_map()
        spawn_points = map.get_spawn_points()
        
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.02 #25 fps, 4ms
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        blueprints_vehicle = world.get_blueprint_library().filter("vehicle.*")
        # sort the vehicle list by id
        blueprints_vehicle = sorted(blueprints_vehicle, key=lambda bp: bp.id)
        # number_of_spawn_points = len(spawn_points)
        # nubmer_of_vehicles = number_of_spawn_points - 1
        nubmer_of_vehicles = 50
        
        # 默认端口8000
        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)
        # tm里的每一辆车都要和前车保持至少3m的距离来保持安全
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        # tm里面的每一辆车都是混合物理模式
        traffic_manager.set_hybrid_physics_mode(True)
        # tm里面每一辆车都是默认速度的80%
        traffic_manager.global_percentage_speed_difference(-50)
        
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        
        batch = []
        
        for n, transform in enumerate(spawn_points):
            # 迭代次数超过预设的TM管理的车辆数
            if n >= nubmer_of_vehicles:
                break
            
            blueprint = random.choice(blueprints_vehicle)
            
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            
            # set autopilot
            blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot all together
            batch.append(SpawnActor(blueprint, transform)
                         .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))    
        
        # execute the command
        for (i, response) in enumerate(client.apply_batch_sync(batch, synchronous_master)):
            if response.error:
                logging.error(response.error)
            else:
                print("Future Actor", response.actor_id)
                vehicles_id_list.append(response.actor_id)
        
        vehicles_list = world.get_actors().filter("vehicle.*")
        
        world.tick()
        
        # set several of the cars as dangerous car
        for i in range(5):
            danger_car = vehicles_list[i]
            # crazy car ignore traffic light, do not keep safe distance, and very fast
            traffic_manager.ignore_lights_percentage(danger_car, 100)
            traffic_manager.distance_to_leading_vehicle(danger_car, 0)
            traffic_manager.vehicle_percentage_speed_difference(danger_car, -50)
        
        print('spawned %d vehicles , press Ctrl+C to exit.' % (len(vehicles_list)))
        
        # set weather for your world
        weather = carla.WeatherParameters(cloudiness=10.0,
                                  precipitation=10.0,
                                  fog_density=10.0)
        world.set_weather(weather)
        # set model3 as ego
        blueprint_library = world.get_blueprint_library()
        # retrieve model3
        ego_bp = blueprint_library.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','0,255,0')
        ego_bp.set_attribute('role_name','hero')
        # get a valid transform that has not been assigned yet
        # TM管理的车的数量是vehicles_id_list - 1个，所以最后一个有效的位置是留给自车的
        ego_spectator_transform = spawn_points[len(vehicles_id_list)]
        
        # ego_transform = random.choice(spawn_points)
        # print(ego_transform)
        # spawn ego actor
        ego_actor = world.try_spawn_actor(ego_bp, ego_spectator_transform)
        ego_actor.set_autopilot(True)
        vehicles_id_list.append(ego_actor.id)
        actor_list.append(ego_actor)

        sensor_queue = Queue(maxsize=10)
        
        # set sensors
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_actor)
        # camera.listen(lambda image: sensor_callback(image, sensor_queue, 'camera'))
        sensor_list.append(camera)
        
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_location = carla.Location(0, 0, 2)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to = ego_actor)
        # lidar.listen(lambda point_cloud: sensor_callback(point_cloud, sensor_queue, 'lidar'))
        sensor_list.append(lidar)
        
        while True:
            
            # set the sectator to follow the ego vehicle
            spectator = world.get_spectator()
            ego_spectator_transform = ego_actor.get_transform()
            spectator.set_transform(carla.Transform(ego_spectator_transform.location + carla.Location(z=20),carla.Rotation(pitch=-90)))
            world.tick()
            # As the queue is blocking, we will wait in the queue.get() methods until all the infomation is processed and 
            # we continue with the next frame.
            # try:
            #     for i in range(len(sensor_list)):
            #         s_frame = sensor_queue.get(True, 1.0)
            #         print("Frame: %d Sensor: %s" % (s_frame[0], s_frame[1]))
            # except Empty:
            #     print("Some of the sensor infomation is missed")
        
    finally:
        world.apply_settings(original_settings)
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user')