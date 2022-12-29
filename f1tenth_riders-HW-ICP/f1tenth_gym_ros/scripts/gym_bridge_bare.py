#!/usr/bin/env python3
import rospy
import os
import numpy as np
import gym
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import quaternion_from_euler
from f1tenth_gym_ros.msg import Observation

current_dir = os.path.abspath(os.path.dirname(__file__))
package_dir = os.path.abspath(os.path.join(current_dir, ".."))

RENDERING_ENABLED = os.environ.get("F1TENTH_RENDERING", "False").lower() == "true"


def _to_odometry_msg(pose_x, pose_y, pose_theta, linear_vel_x, linear_vel_y, angular_vel_z, **kwargs):
    pose = PoseWithCovariance()
    pose.pose.position.x = pose_x
    pose.pose.position.y = pose_y
    quat = quaternion_from_euler(0., 0., pose_theta)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    twist = TwistWithCovariance()
    twist.twist.linear.x = linear_vel_x
    twist.twist.linear.y = linear_vel_y
    twist.twist.angular.z = angular_vel_z
    return pose, twist


class Agent(object):
    def __init__(self, id, drive_callback, scan_fov, scan_beams):
        self.id = id

        # params
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams

        # observations
        self.collision = False
        self.ego_obs = {}

        # motors
        self.requested_steer = 0.0
        self.requested_speed = 0.0
        self.drive_published = False

        #
        self.scan_topic = '/%s/scan' % self.id
        self.observations_topic = '/%s/observations' % self.id
        self.drive_topic = '/%s/drive' % self.id
        self.observations_pub = rospy.Publisher(self.observations_topic, Observation, queue_size=1)
        self.drive_sub = rospy.Subscriber(self.drive_topic, AckermannDriveStamped, drive_callback, queue_size=1)
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)

    def update_observersations(self, ego_obs):
        self.ego_obs = ego_obs
        self.collision = ego_obs['collision']

    def publish_observations(self, ts):
        observation = Observation()
        observation.header.stamp = ts
        observation.header.frame_id = 'agent_%s/observation' % self.id
        observation.ranges = self.ego_obs['scan']

        # calculate pose & twist
        observation.ego_pose, observation.ego_twist = _to_odometry_msg(**self.ego_obs)

        # send!
        self.observations_pub.publish(observation)

        # FIXME: Send /%/scan too.
        def generate_scan_message(name, ranges):
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = '%s/laser' % name
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 30.
            scan.ranges = ranges
            return scan

        scan = generate_scan_message(self.id, self.ego_obs['scan'])
        try:
            self.scan_pub.publish(scan)
        except Exception as e:
            pass


class BridgePlugin:

    def __init__(self):
        pass

    def on_update(self, agents, obs, done):
        pass

    def on_finish(self, agents, obs, done):
        pass


class InfoPrinterPlugin(BridgePlugin):

    def __init__(self, drive_timer_duration):
        super().__init__()
        self.PRINT_EVERY_SECONDS = 2
        self.PRINT_EVERY_TIMER_TICK = int(self.PRINT_EVERY_SECONDS / drive_timer_duration)
        self.timer_tick_count = 0

    def _print(self, agents, obs, done):
        print("Lap Times", [float(i) for i in obs['lap_times']])
        print("Lap Counts", [int(i) for i in obs['lap_counts']])
        print("Collisions", [int(i) for i in obs['collisions']])
        print("Drive Published", [bool(a.drive_published) for a in agents])

    def on_update(self, agents, obs, done):
        self.timer_tick_count += 1
        if self.timer_tick_count == self.PRINT_EVERY_TIMER_TICK:
            self._print(agents, obs, done)
            self.timer_tick_count = 0

    def on_finish(self, agents, obs, done):
        self._print(agents, obs, done)


class GymBridge(object):
    DRIVE_TIMER_DURATION = 0.02

    def __init__(self, scenario, map_path, ego_id, opp_id=None, map_image_ext=".png", plugins=[]):

        # get env vars
        self.race_scenario = scenario
        self.agents, self.rewards, self.plugins = [], [], plugins

        print("Initializing GymBridge with %s plugins." % len(plugins))

        scan_fov, scan_beams = rospy.get_param('scan_fov'), rospy.get_param('scan_beams')
        ids = [ego_id, opp_id]
        for ix, agent_id in enumerate(ids):
            if not agent_id:
                continue
            self.agents.append(Agent(agent_id, self._drive_callback(ix), scan_fov, scan_beams))

        # Topic Names
        self.race_info_topic = rospy.get_param('race_info_topic')

        # Map
        self.map_path = map_path
        self.map_image_ext = map_image_ext

        # Scan simulation params
        scan_fov = rospy.get_param('scan_fov')
        scan_beams = rospy.get_param('scan_beams')
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams

        print("Initiating Gym...")

        # Launch
        driver_count = len(self.agents)
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.map_path[:-5],
                            map_ext=self.map_image_ext, num_agents=driver_count)

        print("Gym is ready. Resetting environment...")

        # init gym backend
        map_config_filename = os.path.basename(self.map_path)
        if driver_count == 1:
            # TODO: Why don't we read these positions from yaml?
            if 'SOCHI'.lower() in map_config_filename.lower():
                poses = np.array([[0.8007017, -0.2753365, 4.1421595]])
            elif 'Oschersleben'.lower() in map_config_filename.lower():
                poses = np.array([[0.0702245, 0.3002981, 2.79787]])
            else:
                raise ValueError("Initial position is unknown for map '{}'.".format(map_config_filename))
        elif driver_count == 2:
            if 'SOCHI'.lower() in map_config_filename.lower():
                poses = np.array([
                    [0.8007017, -0.2753365, 4.1421595],
                    [0.8162458, 1.1614572, 4.1446321],
                ])
            elif 'Oschersleben'.lower() in map_config_filename.lower():
                poses = np.array([
                    [0.0702245, 0.3002981, 2.79787],
                    [0.9966514, -0.9306893, 2.79787],
                ])
            else:
                raise ValueError("Initial positions are unknown for map '{}'.".format(map_config_filename))
        else:
            raise ValueError("Max 2 drivers are allowed")

        self.obs, self.rewards, self.done, _ = self.env.reset(poses=np.array(poses))

        print("Reset complete")

        self._update()

        print("First update called")

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.004), self._timer_callback)
        self.drive_timer = rospy.Timer(rospy.Duration(self.DRIVE_TIMER_DURATION), self._drive_timer_callback)

    def spin(self):
        print("Starting F1Tenth Bridge")

        # run until challenge completes or ros shuts down
        while not self.done and not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.04)

            if RENDERING_ENABLED or os.environ.get("DISPLAY"):
                self.env.render()

        self._finish()

        print("Shutting down F1Tenth Bridge")

    def _update(self):
        # get keys from observations
        # and redirects them to agents in singular form
        keys = {
            'scans': 'scan',
            'poses_x': 'pose_x',
            'poses_y': 'pose_y',
            'poses_theta': 'pose_theta',
            'linear_vels_x': 'linear_vel_x',
            'linear_vels_y': 'linear_vel_y',
            'ang_vels_z': 'angular_vel_z',
            'collisions': 'collision',
            'lap_times': 'lap_time',
            'lap_counts': 'lap_count'
        }

        # if single agent, we won't have opp obs
        for i in range(len(self.agents)):
            obs = {single: self.obs[multi][i] for multi, single in keys.items()}
            self.agents[i].update_observersations(obs)

        for plugin in self.plugins:
            plugin.on_update(self.agents, self.obs, self.done)

    def _finish(self):
        for plugin in self.plugins:
            plugin.on_finish(self.agents, self.obs, self.done)

    def _drive_callback(self, index):
        def inner_callback(drive_msg):
            self.agents[index].requested_speed = drive_msg.drive.speed
            self.agents[index].requested_steer = drive_msg.drive.steering_angle
            self.agents[index].drive_published = True

        return inner_callback

    def _drive_timer_callback(self, timer):
        if self.done:
            return

        published = all([a.drive_published for a in self.agents])

        # until all agents started publishing, we wait
        if not published:
            return

        # update simulation
        # set speed to 0 after collision
        actions = []
        for a in self.agents:
            if a.collision:
                actions.append([0.0, 0.0])
            else:
                actions.append([a.requested_steer, a.requested_speed])

        self.obs, _, self.done, _ = self.env.step(np.array(actions))

        # update scan data
        self._update()

    def _timer_callback(self, timer):
        # once match is completed, stop publishing information
        if self.done:
            return

        ts = rospy.Time.now()
        for i, agent in enumerate(self.agents):
            agent.publish_observations(ts)


if __name__ == '__main__':
    race_scenario = os.environ.get("RACE_SCENARIO", 0)
    race_map_path = os.environ.get("RACE_MAP_PATH")
    if not race_map_path:
        raise Exception("Map Path is not provided, cannot start simulation")

    ego_id = os.environ.get("EGO_ID")
    opp_id = os.environ.get("OPP_ID")

    plugins = [
        InfoPrinterPlugin(GymBridge.DRIVE_TIMER_DURATION),
    ]

    rospy.init_node('gym_bridge')
    gym_bridge = GymBridge(race_scenario, race_map_path, ego_id, opp_id=opp_id, plugins=plugins)
    gym_bridge.spin()

    # once we're here, we know that competition is completed, so we publish info to API
    time.sleep(1)
