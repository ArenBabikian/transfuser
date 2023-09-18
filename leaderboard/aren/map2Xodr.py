import carla
import argparse
from pathlib import Path

# #### NOTE
# This file is yielding "fatal errors" with low quality and making the simulator crash.
# Just do trial and error and it should be fine

def main():

    #add argument support here, if necessary
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    # client.get_world().get_settings().no_rendering_mode = True

    # Path(f'{save_path}/').mkdir(parents=True, exist_ok=True)

    world = client.get_world()
    m = world.get_map()
    xodr_str = m.to_opendrive()

    with open(f'tools/Town07.xodr', 'w+') as f:
        f.write(xodr_str)
    print("Done.")


if __name__ == '__main__':
    main()