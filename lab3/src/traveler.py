#!/usr/bin/env python3

import rospy
import math
import sys

# filePath = '/home/jjariwala/catkin_ws/src/RBE3002_B24_Team17'
filePath = '/home/riley/catkin_ws/src/RBE3002_B24_Team17'

sys.path.insert(1,filePath)
from path_planner import PathPlanner
from lab2.src.lab2 import Lab2 
from nav_msgs.srv import GetPlan
from typing import Tuple as tuple
from typing import List as list
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion

class traveler:
    def __init__(self) -> None:

        rospy.init_node("traveler")

        self.positionHandler = Lab2()

        self.planning = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.getPathPlanned)


    def getPathPlanned(self, msg: PoseStamped) -> None:

        goal = msg

        start = PoseStamped()
        start.pose.position.x = self.positionHandler.px
        start.pose.position.y = self.positionHandler.py

        message = GetPlan()

        message.start = start
        message.goal = goal

        rospy.wait_for_service('plan_path')
        pathP = rospy.ServiceProxy('plan_path', GetPlan)

        path = Path()
        path = pathP(start, goal, 0.1).plan

        path.poses[len(path.poses)-1].pose.orientation = msg.pose.orientation
        
        # print(path)
        print(path.poses)
        # we are already at pos 0, so no need to "go_to" it
        for i in range(1, len(path.poses)):
            self.positionHandler.go_to(path.poses[i])


    def run(self):
        rospy.spin()

if __name__ ==  '__main__':
    traveler().run()
