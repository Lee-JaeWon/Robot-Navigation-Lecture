import time
import gym
import numpy as np
import concurrent.futures
import os
import sys

# Get ./src/ folder & add it to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(current_dir)

# import your drivers here
from pkg.drivers_HW import GapFollower
# from pkg.drivers import DisparityExtender

# choose your drivers here (1-4)
drivers = [GapFollower()]

# choose your racetrack here (Oschersleben, SOCHI, SOCHI_OBS)
RACETRACK = 'SOCHI_OBS'


def _pack_odom(obs, i):
    keys = {
        'poses_x': 'pose_x',
        'poses_y': 'pose_y',
        'poses_theta': 'pose_theta',
        'linear_vels_x': 'linear_vel_x',
        'linear_vels_y': 'linear_vel_y',
        'ang_vels_z': 'angular_vel_z',
    }
    return {single: obs[multi][i] for multi, single in keys.items()}


class GymRunner(object):

    def __init__(self, racetrack, drivers):
        self.racetrack = racetrack
        self.drivers = drivers

    def run(self):
        # load map
        env = gym.make('f110_gym:f110-v0',
                       map="{}/maps/{}".format(current_dir, RACETRACK),
                       map_ext=".png", num_agents=len(drivers))

        # specify starting positions of each agent
        driver_count = len(drivers)
        if driver_count == 1:
            if 'SOCHI'.lower() in RACETRACK.lower():
                poses = np.array([[0.8007017, -0.2753365, 4.1421595]])
            elif 'Oschersleben'.lower() in RACETRACK.lower():
                poses = np.array([[0.0702245, 0.3002981, 2.79787]])
            else:
                raise ValueError("Initial position is unknown for map '{}'.".format(RACETRACK))
        elif driver_count == 2:
            if 'SOCHI'.lower() in RACETRACK.lower():
                poses = np.array([
                    [0.8007017, -0.2753365, 4.1421595],
                    [0.8162458, 1.1614572, 4.1446321],
                ])
            elif 'Oschersleben'.lower() in RACETRACK.lower():
                poses = np.array([
                    [0.0702245, 0.3002981, 2.79787],
                    [0.9966514, -0.9306893, 2.79787],
                ])
            else:
                raise ValueError("Initial positions are unknown for map '{}'.".format(RACETRACK))
        else:
            raise ValueError("Max 2 drivers are allowed")

        obs, step_reward, done, info = env.reset(poses=poses)
        env.render()

        laptime = 0.0
        start = time.time()

        while not done:
            actions = []
            futures = []
            with concurrent.futures.ThreadPoolExecutor() as executor:
                odom_0, odom_1 = _pack_odom(obs, 0), None
                if len(drivers) > 1:
                    odom_1 = _pack_odom(obs, 1)

                for i, driver in enumerate(drivers):
                    if i == 0:
                        ego_odom, opp_odom = odom_0, odom_1
                    else:
                        ego_odom, opp_odom = odom_1, odom_0
                    scan = obs['scans'][i]
                    if hasattr(driver, 'process_observation'):
                        futures.append(executor.submit(driver.process_observation, ranges=scan, ego_odom=ego_odom))
                    elif hasattr(driver, 'process_lidar'):
                        futures.append(executor.submit(driver.process_lidar, scan))

            for future in futures:
                speed, steer = future.result()
                actions.append([steer, speed])
            actions = np.array(actions)
            obs, step_reward, done, info = env.step(actions)
            laptime += step_reward
            env.render(mode='human')

        print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)


if __name__ == '__main__':
    runner = GymRunner(RACETRACK, drivers)
    runner.run()
