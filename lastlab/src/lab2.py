#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import tf 

SCHMOVE_TOLERANCE = 0.03
MAX_ANGSPEED = 0.57 # 0.5 worked wellish lil slow
MAX_LINSPEAD = 0.105
MAX_PENALTY = 0.1 # 0.1 was fine


MAX_ROTI = 2.0
MAX_DRIVEI = 10.0


class Lab2:
    
    def __init__(self):
        """
        Class constructor
        """

        self.px = 0
        self.py = 0
        self.pth = 0
        self.turnP = 1.3 # 1.3 worked great 

        self.rotP = 0.95 # .55
        self.driveP = 0.475 # 0.55 was goodish

        # NEED I TERM FOR MOVEMENT
        self.rotI = 0.001
        self.driveI = 0.00025
            
        self.pathQueue = Path()
        self.pathIter = 1
        self.newPath = False
        self.stop = False # for communicating a NEW path that needs to be followed
        self.stuck = False
        rospy.init_node('lab2')

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.resumePub = rospy.Publisher("/resume", String, queue_size = 10)
        self.posePub = rospy.Publisher("/currentPosition", Point, queue_size = 10)

        self.tf_listener = tf.TransformListener()
            
        self.odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)
            
        # self.move_base_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        self.navPath = rospy.Subscriber("/navPath", Path, self.navPath)
        self.stopSub = rospy.Subscriber("/STOP", String, self.stopHandler)
        

        rospy.sleep(1.0)
        rospy.loginfo("lab2 node ready")
     
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

        
        # CURRENTLY UNUSED, see smooth drive for up to date implementatin
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # Save the initial pose
        initial_x = self.px
        initial_y = self.py

        # Calculate the distance error
        error = 0.0

        # Set a tolerance for the distance error
        tolerance = 0.015  # large increase from 0.01
        
        while not self.stop and error < distance - tolerance:
            self.send_speed(linear_speed, 0.0)

            current_x = self.px
            current_y = self.py
            dx = current_x - initial_x
            dy = current_y - initial_y
            current_distance = math.sqrt(dx**2 + dy**2)

            error = current_distance
            rospy.sleep(0.01)
        
        self.send_speed(0.0, 0.0)
        print("Done Driving")

        
    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        modAngle = self.bindPi(angle)

        angleGoal = modAngle + self.pth # angle to drive to
        # print("We are:", round(self.pth,4), "AG: ", round(angleGoal,4))
        angleError = abs(angleGoal-self.pth)
        errorMargin = .005 # from 0.01

        while not self.stop and abs(angleError) > errorMargin :
            angleError = math.fmod(angleGoal - self.pth, math.pi)
            
            if(abs(angleError) > errorMargin) :
                actualSpeed = self.bindVal(angleError*self.turnP, aspeed)
                # print("AE: ", round(angleError, 3), " AS:", round(actualSpeed, 3))
                self.send_speed(0,actualSpeed)
                rospy.sleep(0.01)
            else :
                self.send_speed(0,0)
        print("Turning Done")

    def rotateToAngle(self, angleGoal: float):
        actualAngleGoal = self.bindPi(angleGoal)
        # print("We are:", round(self.pth,4), "AG: ", round(actualAngleGoal,4))
        angleError = abs(angleGoal-self.pth)
        errorMargin = .01 # from 0.01

        sumRotError = 0

        while not self.stop and abs(angleError) > errorMargin :
            angleError = math.fmod(actualAngleGoal - self.pth, math.pi)
            
            sumRotError = self.bindVal(sumRotError+angleError, MAX_ROTI)
            ang_speed = angleError*self.rotP + sumRotError*self.rotI

            ang_speed = self.bindVal(ang_speed, MAX_ANGSPEED)
            # print("AS:", round(ang_speed,4), "AE: ", round(angleError,4))
            self.send_speed(0,ang_speed)
            rospy.sleep(0.01)

        self.send_speed(0,0)
        rospy.sleep(1)
        print("Turning Done")

    def stopHandler(self, msg: str):
        self.stop = True
        rospy.loginfo("[From StopHandler]: " + str(msg))


    def navPath(self, msg: Path) :
        rospy.loginfo("Updating Path Data")
        print("Length of queue", len(msg.poses))
        self.pathQueue = msg
        self.newPath = True

    def schmoveTo(self, targetPos: Point) :

        distError = math.dist((self.px, self.py), (targetPos.x, targetPos.y))
        # initDistErr = distError # saved for normaizing purposes

        sumDriveError = 0
        sumRotError = 0

        while not self.stop and distError > SCHMOVE_TOLERANCE:
            
            distError = math.dist((self.px, self.py), (targetPos.x, targetPos.y))
            sumDriveError = self.bindVal(sumDriveError + distError, MAX_DRIVEI)

            targetAngle = math.atan2(targetPos.y-self.py, targetPos.x-self.px)
            angError = self.bindPi(targetAngle-self.pth)
            sumRotError = self.bindVal(sumRotError+angError, MAX_ROTI)

            ang_speed = angError*self.rotP + sumRotError*self.rotI
            ang_speed = self.bindVal(ang_speed,MAX_ANGSPEED)

            # lin_speedPenalty = abs((ang_speed) * (distError/initDistErr)) # normalize against distance away, as angle error bemoces more volatile as distance decreases
            lin_speedPenalty = min(abs((ang_speed)) / 8.0, MAX_PENALTY)
            linear_speed = distError*self.driveP + sumDriveError*self.driveI 

            linear_speed = max( -0.0245, self.bindVal(linear_speed, MAX_LINSPEAD)- lin_speedPenalty)

            self.send_speed(linear_speed, ang_speed)
            rospy.sleep(0.01)
    
        self.send_speed(0.0, 0.0)
        print("Done Driving")

    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound toquat_orig a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        
        initial_x = self.px
        initial_y = self.py
        initial_pth = self.pth
        # Extract the target pose information
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        # Calculate the desired angle for the first rotation

        start_pth = math.atan2(target_y-initial_y,target_x-initial_x)

        delta_yaw = self.bindPi(start_pth - initial_pth)

        if(abs(delta_yaw) > math.pi/3.9) :
            print("Angle too large to Schmoove, T+D instead")
            self.rotateToAngle(self.bindPi(start_pth))
            delta_yaw = abs(self.pth - start_pth)
            if delta_yaw > math.pi / 1.1 :
                self.rotate(delta_yaw, 0.3)

        self.schmoveTo(msg.pose.position)

        # Ensure the robot stops completely
        self.send_speed(0.0, 0.0)


    # def update_odometry(self, msg: Odometry):
    #     """
    #     Updates the current pose of the robot.
    #     This method is a callback bound to a Subscriber.
    #     :param msg [Odometry] The current odometry information.
    #     """
    #     self.tf_listener.waitForTransform('/map','/odom', rospy.Time(0), rospy.Duration(1.0))
    #     (trans,rot) = self.tf_listener.lookupTransform('/map','/odom', rospy.Time(0))
        
    #     new_odom = self.transform_odom(msg, trans, rot)
        
    #     self.px = new_odom.pose.pose.position.x
    #     self.py = new_odom.pose.pose.position.y
    #     orientation = new_odom.pose.pose.orientation
    #     (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    #     self.pth = self.bindPi(yaw)

    #     self.posePub.publish(Point(x=self.px, y=self.py))

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        trans = [0,0]
        rot = [0,0,0,0]
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map','/base_footprint',rospy.Time(0)) 
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("HEY I DIDN'T WORK")
        self.px = trans[0]
        self.py = trans[1]

        quat_list = [rot[0], rot[1], rot[2], rot[3]]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

        self.pth = self.bindPi(yaw)

        self.posePub.publish(Point(x=self.px, y=self.py))

    def transform_odom(self, msg: Odometry, trans: list, rot: list) -> Odometry:
        pose = msg.pose.pose
        twist = msg.twist.twist

        pose.position.x += trans[0]
        pose.position.y += trans[1]
        pose.position.z += trans[2]

        current_quaternion =[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]

        rotation_quaternion = quaternion_from_euler(rot[0],rot[1],rot[2])

        new_quaternion = quaternion_multiply(rotation_quaternion, current_quaternion)

        pose.orientation.x = new_quaternion[0]
        pose.orientation.y = new_quaternion[1]
        pose.orientation.z = new_quaternion[2]
        pose.orientation.w = new_quaternion[3]

        transformed_odom = Odometry()
        transformed_odom.header = msg.header
        transformed_odom.child_frame_id = '/map'
        transformed_odom.pose.pose = pose
        transformed_odom.twist.twist = twist

        return transformed_odom

    def smooth_drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # follow the curve -(distErr - xInit)*(distErr - xFinal) * linearSpeed + base
        initPos = [self.px, self.py]

        goalX = self.px + math.cos(self.pth)*distance
        goalY = self.py + math.sin(self.pth)*distance
        goalPos = [goalX, goalY]
        fullDistError = math.dist(initPos, goalPos)
        distErr = fullDistError
        errorMargin = .02
        proportionalDist = 0
        propErrorMargin = 0.01

        while distErr > errorMargin  and abs(1.0-proportionalDist) > propErrorMargin:
            currentPos = [self.px, self.py]
            distErr = math.dist(currentPos, goalPos)
            vectorErr = [0,0]
            vectorErr[0] = currentPos[0]-initPos[0]
            vectorErr[1] = currentPos[1]-initPos[1]
            curDistProp = math.sqrt(pow(vectorErr[0],2) + pow(vectorErr[1],2))
            proportionalDist = curDistProp / fullDistError # should be from [0,1]
            # print("PropDist:", round(proportionalDist,4)," DE: ", round(distErr,4))
            if(distErr > errorMargin) and abs(1.0-proportionalDist) > propErrorMargin :
                actualSpeed = -proportionalDist*(proportionalDist-1) * 4 * linear_speed + 0.01
                if abs(actualSpeed) > linear_speed :
                    actualSpeed = math.copysign(linear_speed, actualSpeed)
                # print("ActualSpeed:", actualSpeed)
                self.send_speed(actualSpeed,0)
                rospy.sleep(.01)
            else :
                self.send_speed(0,0)
                rospy.sleep(.5)
        print("Driving Done")

    # binds an angle between -pi and pi 
    def bindPi(self, angle: float) :
        bound = math.fmod(angle,2*math.pi)
        if abs(bound) > math.pi :
            bound += math.copysign(2*math.pi,-angle)
        return bound

    # this method binds the input val between +-maxVal and keeps the original sign
    def bindVal(self, val: float, maxVal) -> float:
        return math.copysign(min(abs(val), maxVal), val)

    def run(self):
        print("Lab2 running!")
        # state machine logic for following paths
        while(1) :
            pathLength = len(self.pathQueue.poses)
            if pathLength == 0 : # if we have an empty path, wait for new path
                # "stop" is a command, not a physical state of the robot. 
                # If we are already stopped, we should reset this variable 
                self.stop = False   
                rospy.sleep(0.5)
            elif self.newPath :
                self.pathIter = 1 # reset 
                self.stop = False
                self.newPath = False
            elif self.pathIter >= pathLength : # we are done following path
                print("Done following path")
                self.pathIter = 1 # reset
                self.pathQueue.poses = []
                self.resumePub.publish("resume")
            elif self.stop : 
                print("Handled Stop Condition")
                self.pathIter = 1 # reset iter
                self.pathQueue.poses = []
                self.stop = False
            else :
                print("following path. iter: ", self.pathIter)
                self.go_to(self.pathQueue.poses[self.pathIter])
                self.pathIter = self.pathIter + 1
        

if __name__ == '__main__':
    Lab2().run()