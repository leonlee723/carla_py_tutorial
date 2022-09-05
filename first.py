'''
https://zhuanlan.zhihu.com/p/340031078
'''
import carla
from carla import Transform, Rotation, Location
import random
import numpy as np
from queue import Queue
from queue import Empty


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    if 'camera' in sensor_name:
        sensor_data.save_to_disk('output/%06d.png' % sensor_data.frame)
    sensor_queue.put((sensor_data.frame, sensor_name))

def main():
    sensor_list = []
    try:
        client = carla.Client('localhost',2000)
        client.set_timeout(2.0)

        print("client version:" + client.get_client_version())
        print("server version:" + client.get_server_version())
        maps = client.get_available_maps()
        for map in maps:
            print("maps:" + map)
        # world = client.get_world()
        world = client.load_world('Town05_opt')
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)

        sensor_queue = Queue()

        weather = carla.WeatherParameters(cloudiness=10.0,
                                        precipitation=10.0,
                                        fog_density=10.0)

        world.set_weather(weather)
        # blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
        # for blueprint in blueprints:
        #    print(blueprint.id)
        #    for attr in blueprint:
            #    print('  - {}'.format(attr))
        blueprint_library = world.get_blueprint_library()

        # 从浩瀚如海的蓝图中找到奔驰的蓝图
        ego_vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        pedestrain10_bp = blueprint_library.find('walker.pedestrian.0010')
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '110')
        # Set the time in seconds between sensor captures
        camera_bp.set_attribute('sensor_tick', '1.0')
        # 给我们的车加上特定的颜色
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')

        # 找到所有可以作为初始点的位置并随机选择一个
        ego_transform = random.choice(world.get_map().get_spawn_points())
        # 在这个位置生成汽车
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_transform)
        ego_vehicle.set_autopilot(True)
        # relative_transform = ego_vehicle.get_transform()
        # relative_transform.location.z += 10
        relative_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        # camera = world.spawn_actor(camera_bp,relative_transform, attach_to=ego_vehicle, attachment=carla.AttachmentType.Rigid)
        camera = world.spawn_actor(blueprint=camera_bp,transform=relative_transform, attach_to=ego_vehicle)

        camera.listen(lambda image: sensor_callback(image, sensor_queue, "camera"))
        sensor_list.append(camera)

        pedestrain10_location = world.get_random_location_from_navigation()
        # pedestrain10_transform = Transform(pedestrain10_location, Rotation(yaw=180))
        pedestrain10_transform = carla.Transform()
        pedestrain10_transform.location = pedestrain10_location
        pedestrain10 = world.spawn_actor(pedestrain10_bp, pedestrain10_transform)

       
        while True:
            world.tick()
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),carla.Rotation(pitch=-90)))
            
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    print("     Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))
            except Empty:
                print("   Some of the sensor information is missed")

    finally:
        # world.apply_settings(original_settings)
        # print('destroying actors')
        # client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')

    


# 再给它挪挪窝
# location = ego_vehicle.get_location()
# location.x += 10.0
# ego_vehicle.set_location(location)
# 把它设置成自动驾驶模式
# ego_vehicle.set_autopilot(True)
# 我们可以甚至在中途将这辆车“冻住”，通过抹杀它的物理仿真
# actor.set_simulate_physics(False)

# 如果注销单个Actor
# ego_vehicle.destroy()
# 如果你有多个Actor 存在list里，想一起销毁。
# client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')