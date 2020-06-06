#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid # import occupancy grid data
from sparse_rrt.planners import SST
from sparse_rrt.systems.FreePoint import FreePoint
import cv2

"""
TO DO
publishing path and test with rviz or something
upgrade creating obs 
implement car model instead of point (in FreePoint)
"""

map_cpy = OccupancyGrid()

def callback(occ_map):
   # copy map from /map topic to map_cpy variable
   map_cpy.header = occ_map.header
   map_cpy.info = occ_map.info
   map_cpy.data = occ_map.data


def get_data_system(map_cpy):
    print('get data')
    map_data = {}
    h = map_cpy.info.width
    w = map_cpy.info.height
    map_data["min_x"] = 0.0
    map_data["max_x"] = w
    map_data["min_y"] = 0.0
    map_data["max_y"] = h
    obstacles = np.array([[0, 0, 0, 0]])
    # detect obstaces
    print('create obstacles')
    for i in range(w):
        for j in range(h):
            if map_cpy.data[w * i + j] == 100:
                # create obstacle
                obs = np.zeros((1, 4))
                # left_x
                obs[0][0] = j
                # bottom_y
                # obs[0][1] = ((h-i) - 1)
                obs[0][1] = i
                # right_x
                obs[0][2] = j + 1
                # top_y
                # obs[0][3] = (h - i)
                obs[0][3] = i + 1
                # print(obs)
                obstacles = np.append(obstacles, obs, axis=0)
                # print(obs)
                # np.concatenate((obstacles, obs), axis=0)
    return map_data, obstacles, w, h

def path_drawing(solution, rapper, w, h):
    obs = solution[0]
    print(w, "   ", h)
    for j in range(len(solution) - 2):
        for i in range(len(obs) - 1):
            p1 = solution[j][i]
            p2 = solution[j][i + 1]
            point1x = int((p1[0]) * 500 / w)
            point1y = int((h - (p1[1])) * 500 / h)
            point2x = int((p2[0]) * 500 / w)
            point2y = int((h - (p2[1])) * 500 / h)
            coord1 = (point1x, point1y)
            coord2 = (point2x, point2y)
            cv2.line(rapper, coord1, coord2, (255, 0, 0), 2)
    return rapper


def mapListener():
    #create node
    rospy.init_node('map_listener', anonymous=True)
    #subscribe /map topic
    print('XXXXXXXXXXXXXXXXXX')
    print('geting data from rosnode')
    w = 0
    h = 0

    rospy.Subscriber("map", OccupancyGrid, callback)
    # spin() simply keeps python from exiting until this node is stopped
    pub = rospy.Publisher('mapaaaa', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # 1hz
    start_experiment = False
    if_execute = True
    # variable for future, to keep path to publish
    path = []
    while not rospy.is_shutdown():
        #publish copy of the map
        pub.publish(map_cpy)
        map_data = {}
        if len(map_cpy.data) > 0 and len(path) == 0:
            map_data, obstacles, w, h = get_data_system(map_cpy)
            start_experiment = True


        if start_experiment == True and if_execute == True:
             # start the experiment
             if_execute = False
             print('##################################')
             print('setup system')
             system = FreePoint(obstacles, obstacles.shape[0], map_data)
             #create sst planner
             print(obstacles.shape[0])
             planner = SST(
                state_bounds=system.get_state_bounds(),
                control_bounds=system.get_control_bounds(),
                distance=system.distance_computer(),
                start_state=np.array([2., 15.]),
                goal_state=np.array([15., 15.]),
                goal_radius=0.5,
                random_seed=0,
                sst_delta_near=0.4,
                sst_delta_drain=0.2
             )
             """
             * for map.yaml start and end points might be: 
             * start_state=np.array([2., 15.])
             * goal_state=np.array([15., 15.])
             """
             cv2.namedWindow('map')
             print('start the experiment')
             for iteration in range(20000):
                planner.step(system, 2, 20, 0.1)
                if iteration % 100 == 0:
                   solution = planner.get_solution()
                   print("Solution: %s, Number of nodes: %s" % (planner.get_solution(), planner.get_number_of_nodes()))
             rapper = planner.visualize_nodes(system)
             solution = planner.get_solution()
             if len(solution[0])>0:
                 # draw path if the solution exist
                 path = solution[0]
                 rapper = path_drawing(solution, rapper, w, h)
             q = ord('a')
             while q != ord('q'):
                cv2.imshow('map', rapper)
                q = cv2.waitKey(30)
             cv2.destroyAllWindows()
        rate.sleep()

if __name__ == '__main__':
   mapListener()
