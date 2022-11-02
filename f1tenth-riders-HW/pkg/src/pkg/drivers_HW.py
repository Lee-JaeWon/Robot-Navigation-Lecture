# -*- coding:utf-8 -*-
import numpy as np


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

        # numpy mask 배열 사용
        # 센서가 데이터를 기록하지 못하거나 유효하지 않은 값을 기록하는 등이 있다.
        # numpy.ma 모듈은 마스크 배열을 도입하여 이런 문제를 해결할 편리한 방법을 제시

        # mask the bubble
        # 0을 제외한 값 -> masked
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)

        # get a slice for each contigous sequence of non-bubble data
        # 주어진 축을 따라 마스킹된 배열에서 마스킹되지 않은 연속 데이터를 찾는다.
        slices = np.ma.notmasked_contiguous(masked)
        # print slices -> [slice(0, 649, None), slice(809, 810, None)]
        
        # 최대 길이 계산
        max_len = slices[0].stop - slices[0].start
        
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        
        # slices[0] 이외에 더 큰 length가 있다면 교환
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start

            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
            
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners

        # ranges와 convolution한 length 출력값 반환 : 'same' 모드
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE

        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        # print(f"range_index:{range_index}")
        # print(f"range_len:{range_len}")
        # print(f"self.radians_per_elem:{self.radians_per_elem}")

        # range_index -> best point ridar index
        # best point에 대한 차체의 각도 for steering
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem

        # For ackerman steering model
        steering_angle = lidar_angle / 2

        return steering_angle

    def process_lidar(self, ranges):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(ranges)
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: 
	        min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED
        # print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        return speed, steering_angle


# drives straight ahead at a speed of 5
class SimpleDriver:

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle




