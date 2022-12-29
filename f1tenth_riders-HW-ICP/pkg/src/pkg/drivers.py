import numpy as np
import math
from math import pi
import matplotlib.pyplot as plt 
# from scipy.spatial.transform import Rotation as R_scipy

class GapFollower:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 9.0
    CORNERS_SPEED = 6.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self):
        # used when calculating the angles of the LiDAR data
        self.radians_per_elem = None

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        proc_ranges = np.array(ranges[135:-135])
        # sets each value to the mean over a given window
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len >max_len:
                max_lenc= sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def process_lidar(self, ranges):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        # print(len(ranges));print('\n\n\n\n') --> 1080

        proc_ranges = self.preprocess_lidar(ranges) # --> len : 810

        # print(proc_ranges);print('\n\n\n\n')
        # print(len(proc_ranges));print('\n\n\n\n')
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)
        # print(best)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED
        # print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        return speed, steering_angle

class CustomSLAM:
    '''
    class CustomSLAM in drivers.py
    [1] LiDAR 센서로 부터 거리와 각도 데이터(scan data)를 측정하고 polar coordinate로 구성된 데이터를 Eucliean coordinate로 변환한다.

    [2] [1]에서 얻어진 Euclidean coordinate 상의 데이터는 robot의 위치가 중심인 robot local coodinate이므로 이를 현재 로봇의 추정 pose를 이용하여 global coordinate로 변환한다.

    [3] 바로 이전에 얻어진 global coordinate에서의 scan data와 현재 얻어진 scan data를 정합(ICP를 통해서)하여 두 pose간 relative transform을 계산한다
    '''
    
    def __init__(self):
        # 지도는 map, 위치는 L_pose에 저장할 것
        self.map = []
        self.L_pose = []

        # ICP Algorithm class in ICP.py
        from pkg.ICP import ICP
        self.icp = ICP()

        self.cur_x_data = []
        self.cur_y_data = []
        self.pre_x_data = []
        self.pre_y_data = []
        self.ang = []
        self.simulation_flag = False
        self.n = 0
        self.pre_scan = None

        self.all_R_mat = []
        self.all_T_mat = []

        self.matmul_R = []

        self.all_theta_list = []
        self.all_dist_list = []
        self.flag = 1
        for i in range(360):
            self.ang.append(i)
        self.x_data_pre = None
        self.y_data_pre = None

        # For make Odometry
        self.init_pose_x = 0; self.init_pose_y = 0
        self.plot_cord_x = [];self.plot_cord_y = []
        self.plot_cord_x.append(self.init_pose_x)
        self.plot_cord_y.append(self.init_pose_y)

        self.count = 0
        self.result_pose = []
        self.temp_current_pose = np.matrix([[self.init_pose_x],[self.init_pose_y]])

    def get_scan_data(self, cur_scan, speed, steer):
        '''
        get Current and Previous 2D laser scan data
        '''
        # 현재 Laser scan data
        self.cur_scan = cur_scan 
        
        # LiDAR 거리 데이터를 Cartesian 좌표계로 변환
        x_data_cur, y_data_cur = self.lidar_to_cartesian(self.cur_scan)
        
        # 초기 한번은 이전 값 강제 저장
        if self.flag == 1:
            self.x_data_pre = x_data_cur
            self.y_data_pre = y_data_cur
            # print(self.x_data_pre, self.y_data_pre);print('flag1')
        
        if self.n == 2: # n iteration마다 동작하게 함 --> 계산량으로 인해 매 loop마다 계산하면 오래걸림
            R, T = self.icp.main(x_data_cur, y_data_cur, self.x_data_pre, self.y_data_pre,
                                speed=speed, steer=steer) # Return R, T matrix
            self.all_R_mat.append(R)
            self.all_T_mat.append(T)
            
            # 이전값 저장하기
            self.x_data_pre = x_data_cur
            self.y_data_pre = y_data_cur
            self.flag = 0
            self.n = 0 # reinit self.n

            # Current scan update : 현재 scan도 R/T를 이용해 업데이트 해주어야 한다고 생각
            self.all_scan = np.matrix([x_data_cur, y_data_cur])
            self.all_scan_RT = R@self.all_scan + np.matrix([[T[0]],[T[1]]])
            self.all_scan_RT = np.array(self.all_scan_RT)
            x_data_cur = self.all_scan_RT[0]
            y_data_cur = self.all_scan_RT[1]
            # print(x_data_cur)
            # 

            self.make_odometry()

        self.n = self.n + 1
            
    def lidar_to_cartesian(self, data):
        '''
        Raw LiDAR data to cartesian space
        '''
        if data is not None:
            x_data = []
            y_data = []
            values = data
            sz = len(values)
            delta = 2 * math.pi * 0.75 / sz # 0.75 곱한 이유는 270도 범위 LiDAR 센서이기 때문.
            resolution = 1
            for i in range(sz):
                val = values[i]
                if val >= 0 and val <= 1000:
                    rad = i * delta + math.pi / 2
                    x = resolution * val * math.cos(rad) 
                    y = resolution * val * math.sin(rad)
                    x_data.append(x)
                    y_data.append(y)
            return x_data, y_data

    def make_odometry(self):
        '''
        두 Rotation 혹은 Translation matrix 간 각도 및 거리 구한 결과로 Odometry 만들기
        '''
        # 1차 시도
        # if length == 2:
        #     self.plot_cord_x.append(self.plot_cord_x[0] + (self.all_dist_list[0] * math.cos(self.all_theta_list[0])))
        #     self.plot_cord_y.append(self.plot_cord_y[0] + (self.all_dist_list[0] * math.sin(self.all_theta_list[0])))
        # else: # length 3부터
        #     # 이전 좌표 + 구한 좌표
        #     self.plot_cord_x.append(self.plot_cord_x[length-3] + (self.all_dist_list[length-2] * (math.cos(self.all_theta_list[length-2]))))
        #     self.plot_cord_y.append(self.plot_cord_y[length-3] + (self.all_dist_list[length-2] * (math.sin(self.all_theta_list[length-2]))))
        #     # print("difference")
        #     # print(self.all_theta_list[length-2] - self.all_theta_list[length-3])
        # # print(f"length : {length}")
        # # print(self.plot_cord_x, self.plot_cord_y)

        # # plt.plot(self.plot_cord_x, self.plot_cord_y)
        # # plt.title('Odometry')
        # # plt.draw()
        # # plt.pause(1e-6)
        # # plt.cla()

        if self.count <= len(self.all_R_mat):
            temp_T_mat = np.matrix([[self.all_T_mat[self.count][0]],[self.all_T_mat[self.count][1]]])

            self.temp_current_pose = self.all_R_mat[self.count]@self.temp_current_pose + temp_T_mat

            self.result_pose.append(np.array(self.temp_current_pose))
            self.plot_cord_x.append(self.temp_current_pose[0])
            self.plot_cord_y.append(self.temp_current_pose[1])
            

        self.count = self.count + 1
        
        # plt.plot(self.plot_cord_x, self.plot_cord_y)
        # plt.title('Odometry')
        # plt.draw()
        # plt.pause(1e-6)
        # plt.cla()

    def save_map_pose(self):
        '''
        위의 map, L_pose를 저장하는 함수
        '''
        self.L_pose = self.result_pose
        np.save('./L_Pose.npy', self.L_pose)
        print("L_pose was saved")
