
import argparse
import sys
import glob
import os

# try:
#     # print(os.path.exists('C:/Applications/CARLA_0.9.10.1/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.10-py3.7-win-amd64.egg'))
#     sys.path.append(glob.glob('C:/Applications/CARLA_0.9.10.1/WindowsNoEditor/PythonAPI/carla/dist/carla-0.9.10-py3.7-win-amd64.egg'))
# except IndexError:
#     print('ERROR')
#     pass

import carla

def main():
    parser = argparse.ArgumentParser(description='Script to handle --logdir command line argument.')
    parser.add_argument('--logpath', type=str, help='Directory for log files', default='DEFAULT')
    args = parser.parse_args()

    client = carla.Client("localhost", 2000, worker_threads=1)
    client.set_timeout(5.0)
    world = client.get_world()

    # actor_list = world.get_actors().filter('vehicle.*')
    # print(actor_list)
    # # exit()
    # for actor in actor_list:
    #     print(actor)
    #     actor.destroy()
    # # exit()
    
    # world.set_weather(getattr(carla.WeatherParameters, 'CloudySunset'))


    filepath = args.logpath   
    f = client.show_recorder_file_info(filepath, True)
    f_false = client.show_recorder_file_info(filepath, False)
    print(type(f))
    # exit()
    print(f_false)
    # print(client.show_recorder_collisions(filepath, 'aa', 'aa'))
    exit()

    x = client.replay_file(filepath, 0, 0, 0, 1)
    # spectator_transform=carla.Transform(
    #     carla.Location(x=83.656036, y=243.203705, z=208.359161), 
    #     carla.Rotation(pitch=-88.593880, yaw=-0.327483, roll=-0.000276))
    # print(world.get_spectator().set_transform(spectator_transform))
    print(x)

    #TODO
    # CARLA crashing after scenario replay is finishing
    # Probably has to do with scenario replayer not being stopped correctly

if __name__ == '__main__':
    main()