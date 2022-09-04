'''
https://zhuanlan.zhihu.com/p/340031078
'''
import carla
import random
import numpy as np

client = carla.Client('localhost',2000)
client.set_timeout(2.0)

print("client version:" + client.get_client_version())
print("server version:" + client.get_server_version())
maps = client.get_available_maps()
for map in maps:
    print("maps:" + map)
# world = client.get_world()
world = client.load_world('Town05_opt')
weather = carla.WeatherParameters(cloudiness=10.0,
                                  precipitation=10.0,
                                  fog_density=10.0)

world.set_weather(weather)
# blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
# for blueprint in blueprints:
#    print(blueprint.id)
#    for attr in blueprint:
#        print('  - {}'.format(attr))
# blueprint_library = world.get_blueprint_library()
# 从浩瀚如海的蓝图中找到奔驰的蓝图
# ego_vehicle_bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
# 给我们的车加上特定的颜色
# ego_vehicle_bp.set_attribute('color', '0, 0, 0')

# 找到所有可以作为初始点的位置并随机选择一个
# transform = random.choice(world.get_map().get_spawn_points())
# 在这个位置生成汽车
# ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)

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