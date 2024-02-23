#!/usr/bin/env python3

import rospy
from path_planner import PathPlanner
from lab2 import Lab2
from nav_msgs.msg import GridCells, OccupancyGrid, Path, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from lastlab.srv import GetGridCells, GetPlan2
from geometry_msgs.msg import Twist, PoseArray
from tf.transformations import euler_from_quaternion

from std_srvs.srv import Empty
import tf

from phase3helper import Phelper

class Phase3:

    def __init__(self):
        rospy.init_node("phase3")
        self.covariance = 0

        self.nav = Phelper()
        self.localized = False

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.currNodePub = rospy.Publisher('/currNode', GridCells, queue_size=10)

        self.tf_listener = tf.TransformListener()

        self.amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_handler)
        self.finalPathPub = rospy.Publisher("/FinalPath", GridCells, queue_size=10)

        self.nav_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.getPathPlanned)

        rospy.sleep(1.0)
        rospy.loginfo("phase 3 ready")

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # Create twist message
        msg_cmd_vel = Twist()
        # Linear velocity 
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.angular.z = angular_speed

        self.cmd_vel.publish(msg_cmd_vel)

    def getPathPlanned(self, msg: PoseStamped) -> None:
        print("got nav goal")
        goal = msg

        start = PoseStamped()
        start.pose.position.x = self.nav.px
        start.pose.position.y = self.nav.py

        message = GetPlan2()

        message.start = start
        message.goal = goal

        rospy.wait_for_service('plan_path')
        pathP = rospy.ServiceProxy('plan_path', GetPlan2)
        print("got plan path")

        rospy.wait_for_service('static_map')
        map = rospy.ServiceProxy('static_map', GetMap)
        mapdata = map().map
        print("got static map")

        rospy.wait_for_service('get_Frontier')
        c_space = rospy.ServiceProxy('get_Frontier', GetGridCells)
        mapdata = c_space(mapdata).map

        print("got c space")

        path = Path()
        path = pathP(start, goal, 0.1, mapdata).plan

        path.poses[len(path.poses)-1].pose.orientation = msg.pose.orientation

        pathCells = []
        for i in range (0, len(path.poses)):
            pathCells.append(path.poses[i].pose.position)

        finalPathGC = GridCells()
        finalPathGC.cell_height = mapdata.info.resolution / 1.3
        finalPathGC.cell_width = mapdata.info.resolution / 1.3
        finalPathGC.header = mapdata.header
        finalPathGC.cells = pathCells
        self.finalPathPub.publish(finalPathGC)

        print(path.poses)
        currGC = GridCells()
        currGC.header = mapdata.header
        currGC.cell_height = mapdata.info.resolution
        currGC.cell_width = mapdata.info.resolution
        currGC.cells = []
        # we are already at pos 0, so no need to "go_to" it
        for i in range(1, len(path.poses)):
            currGC.cells = [path.poses[i].pose.position]
            self.currNodePub.publish(currGC)
            self.nav.go_to(path.poses[i])

    def amcl_handler(self, msg: PoseWithCovarianceStamped):
        self.covariance = msg.pose.covariance
        self.nav.px = msg.pose.pose.position.x
        self.nav.py = msg.pose.pose.position.y
        rot = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        quat_list = [rot[0], rot[1], rot[2], rot[3]]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.nav.pth = self.nav.bindPi(yaw)

        # TODO, FIND "good enough" localization params
        print("Zero: ", round(self.covariance[0], 5))
        print(round(self.covariance[1], 5), round(self.covariance[7], 5), round(self.covariance[35], 5))
        if(abs(self.covariance[1]) < 0.004 and abs(self.covariance[7]) < 0.08 and abs(self.covariance[35]) < 0.033):
            if not self.localized :
                print("yay!")
                self.localized = True
    
        # print(round(self.nav.px, 4), round(self.nav.py, 4))

    def spinCycle(self, aspeed) :
        for i in range(0, 1000) :
            if(self.localized) :
                break
            self.send_speed(0.0, aspeed)
            rospy.sleep(0.1)
        self.send_speed(0,0)

    def backForward(self, lspeed) :
        halfTime = 1.5
        self.send_speed(lspeed, 0.0)
        rospy.sleep(halfTime)
        self.spinCycle(0.33)
        self.send_speed(-lspeed, 0)
        rospy.sleep(2*halfTime)
        self.spinCycle(0.33)
        self.send_speed(lspeed, 0.0)
        rospy.sleep(halfTime)
        self.send_speed(0,0)

        pass

    def run(self):
        rospy.wait_for_service("global_localization")
        globalLoc = rospy.ServiceProxy("global_localization", Empty)
        globalLoc()

        # rospy.sleep(15)
        self.localized = False
        while(not self.localized):
            self.spinCycle(0.6)
            # self.spinCycle(-0.5)
            # self.backForward(0.065)
        print( "position: (", self.nav.px, ",", self.nav.py,")", "orientation: ", self.nav.pth)
        rospy.spin()

if __name__ ==  '__main__':
    Phase3().run()