#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Lab2:
    
    def __init__(self):
        """
        Class constructor
        """

        self.px = 0
        self.py = 0
        self.pth = 0
        self.turnP = 1.2
            
        # rospy.init_node('lab2')
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
            
        self.odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)
            
        # self.move_base_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
     
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
        tolerance = 0.01  # need to test
        
        while error < distance - tolerance:
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
        errorMargin = .01
        while abs(angleError) > errorMargin :
            angleError = math.fmod(angleGoal - self.pth, math.pi)
            
            if(abs(angleError) > errorMargin) :
                actualSpeed = angleError*self.turnP
                if abs(actualSpeed) > aspeed :
                    actualSpeed = math.copysign(aspeed,actualSpeed)
                # print("AE: ", round(angleError,4), "\nAS:", round(actualSpeed,4))
                self.send_speed(0,actualSpeed)
                rospy.sleep(0.01)
            else :
                self.send_speed(0,0)
                rospy.sleep(1)
        print("Turning Done")


    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound toquat_orig a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        ANG_SPEED = .4
        LIN_SPEED = .35
        
        initial_x = self.px
        initial_y = self.py
        initial_pth = self.pth
        # Extract the target pose information
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y

        # Extract the orientation of the target pose
        target_orientation = msg.pose.orientation

        # Calculate the desired orientation (yaw) of the robot to face the target
        _, _, target_pth = euler_from_quaternion([target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w])

        # Calculate the desired angle for the first rotation

        start_pth = math.atan2(target_y-initial_y,target_x-initial_x)

        delta_yaw = start_pth - initial_pth

        # Perform the first rotation to face the target
        self.rotate(delta_yaw, ANG_SPEED)

        # Calculate the desired linear distance to reach the target pose
        delta_x = target_x - initial_x
        delta_y = target_y - initial_y
        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Perform the forward movement to reach the target pose

        self.smooth_drive(distance, LIN_SPEED/3) # this was bad for small distances
        # self.drive(distance,LIN_SPEED/3)

        # Perform the second rotation to correct the final orientation

        delta_yaw_final = target_pth - self.pth

        # Perform the second rotation to attain the final orientation
        self.rotate(delta_yaw_final, ANG_SPEED)

        # Ensure the robot stops completely
        self.send_speed(0.0, 0.0)




    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.pth = self.bindPi(yaw)
        # print("updated odom! (", round(self.px,4), ",", round(self.py,4), ",", round(self.theta,4), ")")


    def smooth_drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # follow the curve -(distErr - xInit)*(distErr - xFinal) * linearSpeed + base
        ### EXTRA CREDIT
        # TODO
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
            print("PropDist:", round(proportionalDist,4)," DE: ", round(distErr,4))
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

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Lab2().run()