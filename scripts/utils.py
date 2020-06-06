import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid # import occupancy grid data
from sparse_rrt.planners import SST
from sparse_rrt.systems.FreePoint import FreePoint
import cv2


def wait():
   pass