#!/usr/bin/python
# -*- coding:utf-8 -*-
import sys
import time
import pickle
import numpy as np
import random
import cv2

from itertools import product
from math import cos, sin, pi, sqrt 

from plotting_utils import draw_plan_rrt, draw_plan
from priority_queue import priority_dict

class State(object):
    """
    2D state. 
    """
    
    def __init__(self, x, y, parent):
        """
        x represents the columns on the image and y represents the rows,
        Both are presumed to be integers
        """
        self.x = x
        self.y = y
        self.parent = parent
        self.children = []

        
    def __eq__(self, state):
        """
        When are two states equal?
        """    
        return state and self.x == state.x and self.y == state.y 

    def __hash__(self):
        """
        The hash function for this object. This is necessary to have when we
        want to use State objects as keys in dictionaries
        """
        return hash((self.x, self.y))
    
    def euclidean_distance(self, state):
        assert (state)
        return sqrt((state.x - self.x)**2 + (state.y - self.y)**2)
    
class RRTPlanner(object):
    """
    Applies the RRT algorithm on a given grid world
    """
    
    def __init__(self, world):
        # (rows, cols, channels) array with values in {0,..., 255}
        self.world = world

        # (rows, cols) binary array. Cell is 1 iff it is occupied
        self.occ_grid = self.world[:,:,0]
        self.occ_grid = (self.occ_grid == 0).astype('uint8')
        
    def state_is_free(self, state):
        """
        Does collision detection. Returns true iff the state and its nearby 
        surroundings are free.
        """
        return (self.occ_grid[state.y-5:state.y+5, state.x-5:state.x+5] == 0).all()


    def sample_state(self):
        """
        Sample a new state uniformly randomly on the image. 
        """
        #TODO: make sure you're not exceeding the row and columns bounds
        # x must be in {0, cols-1} and y must be in {0, rows -1}

        # Random 값 생성
        x = random.randint(0, self.world.shape[1]-1) # {0, cols-1}
        y = random.randint(0, self.world.shape[0]-1) # {0, rows-1}
        return State(x, y, None)
           

    def _follow_parent_pointers(self, state):
        """
        Returns the path [start_state, ..., destination_state] by following the
        parent pointers.
        """
        
        curr_ptr = state
        path = [state]
        
        while curr_ptr is not None:
            path.append(curr_ptr)
            curr_ptr = curr_ptr.parent

        # return a reverse copy of the path (so that first state is starting state)
        return path[::-1]


    def find_closest_state(self, tree_nodes, state):
        min_dist = float("Inf")
        closest_state = None
        for node in tree_nodes:
            dist = node.euclidean_distance(state)  
            if dist < min_dist:
                closest_state = node
                min_dist = dist

        return closest_state

    def steer_towards(self, s_nearest, s_rand, max_radius):
        """
        Returns a new state s_new whose coordinates x and y
        are decided as follows:
        
        If s_rand is within a circle of max_radius from s_nearest
        then s_new.x = s_rand.x and s_new.y = s_rand.y
        
        Otherwise, s_rand is farther than max_radius from s_nearest. 
        In this case we place s_new on the line from s_nearest to
        s_rand, at a distance of max_radius away from s_nearest.
        
        """

        #TODO: populate x and y properly according to the description above.
        #Note: x and y are integers and they should be in {0, ..., cols -1}
        # and {0, ..., rows -1} respectively

        # 랜덤 점까지의 거리 계산
        distance_rand_nearest = sqrt((s_rand.x - s_nearest.x)**2 + (s_rand.y - s_nearest.y)**2)
        
        # rand(무작위 샘플점)을 발생시킨 후, 가장 가까운 노드 near를 찾는다.
        # near에서 rand 방향으로 연결한 직선상에 일정한 거리 만큼 떨어진 점을 q_new로 선정

        if (distance_rand_nearest <= max_radius): # 최대 도달거리보다 작으면 랜덤 값 그대로 선정
            x = s_rand.x
            y = s_rand.y
        else: # 최대 도달거리보다 크면 일정비율로 줄여서 적용
            t = max_radius / distance_rand_nearest # 최대거리/계산거리 계산
            x = int((1-t)*s_nearest.x + t*s_rand.x) # 비율에 따른 거리로 x,
            y = int((1-t)*s_nearest.y + t*s_rand.y) # y 설정
        
        
        s_new = State(x, y, s_nearest)
        return s_new


    def path_is_obstacle_free(self, s_from, s_to):
        """
        Returns true iff the line path from s_from to s_to
        is free
        """
        assert (self.state_is_free(s_from))
        
        if not (self.state_is_free(s_to)):
            return False

        max_checks = 10
        for i in xrange(max_checks):
            # TODO: check if the inteprolated state that is float(i)/max_checks * dist(s_from, s_new)
            # away on the line from s_from to s_new is free or not. If not free return False
            distance = sqrt((s_from.x - s_to.x)**2 + (s_from.y - s_to.y)**2)
            distance_interpol = float(i) / max_checks * distance

            x = int(s_from.x + distance_interpol)
            y = int(s_from.y + distance_interpol)

            interpolated_state = State(x,y,s_from)
            
            if not self.state_is_free(interpolated_state):
                return False
            
        # Otherwise the line is free, so return true
        return True

    
    def plan(self, start_state, dest_state, max_num_steps, max_steering_radius, dest_reached_radius):
        """
        Returns a path as a sequence of states [start_state, ..., dest_state]
        if dest_state is reachable from start_state. Otherwise returns [start_state].
        Assume both source and destination are in free space.
        """

        # 시작 지점과 종료 지점에 장애물이 있는 지점인지 아닌지 확인
        assert (self.state_is_free(start_state))
        assert (self.state_is_free(dest_state))

        # The set containing the nodes of the tree
        tree_nodes = set()
        tree_nodes.add(start_state)
        
        # image to be used to display the tree
        img = np.copy(self.world)

        plan = [start_state]
        
        for step in xrange(max_num_steps):

            # TODO: Use the methods of this class as in the slides to
            # compute s_new
            
            s_rand = self.sample_state() # 맵 상에서의 랜덤한 점들의 집합
            s_nearest = self.find_closest_state(tree_nodes, s_rand) # 가장 가까운 state 찾기
            s_new = self.steer_towards(s_nearest, s_rand, max_steering_radius) # nearest에서 rand 방향으로 연결한 직선상에 있는 일정 거리에 있는 점을 s_new로 선정

            if self.path_is_obstacle_free(s_nearest, s_new): # 새로운 점이 장애물과 충돌하는 점이 아닌지 판단
                tree_nodes.add(s_new)
                s_nearest.children.append(s_new)

                # If we approach the destination within a few pixels
                # we're done. Return the path.
                if s_new.euclidean_distance(dest_state) < dest_reached_radius:
                    dest_state.parent = s_new
                    plan = self._follow_parent_pointers(dest_state)
                    break
                
                # TODO:plot the new node and edge
                cv2.circle(img, (s_new.x, s_new.y), 3, (255,0,0)) # 노드 표현
                cv2.line(img, (s_nearest.x, s_nearest.y), (s_new.x, s_new.y), (255,0,0)) # 선 표현

            # Keep showing the image for a bit even
            # if we don't add a new node and edge
            cv2.imshow('image', img)
            cv2.waitKey(10)

        draw_plan_rrt(img, plan, bgr=(0,0,255), thickness=3)
        cv2.waitKey(0)
        return [start_state]


    
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Usage: rrt_planner.py occupancy_grid.pkl"
        sys.exit(1)

    pkl_file = open(sys.argv[1], 'rb')
    # world is a numpy array with dimensions (rows, cols, 3 color channels)
    world = pickle.load(pkl_file)
    pkl_file.close()

    rrt = RRTPlanner(world)

    # 시작 지점과 종료 지점
    start_state = State(10, 10, None)
    dest_state = State(300, 500, None)

    # 샘플링 개수가 충분하지 않다면 존재하는 경로를 찾지 못할 수도 있다.
    max_num_steps = 2500 # max number of nodes to be added to the tree     
    max_steering_radius = 20 # pixels # max_steering_radius값이 너무 크면 작은 장애물를 고려 못할 수 있다.
    dest_reached_radius = 40 # pixels
    
    plan = rrt.plan(start_state,
                    dest_state,
                    max_num_steps,
                    max_steering_radius,
                    dest_reached_radius)
    
