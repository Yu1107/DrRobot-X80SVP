#!/usr/bin/env python


import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, atan2, degrees, cos, sin

# Initialize a counter to track waypoints
i = 0
k = 0
quat_tf = [0, 0, 0, 1]
shortok = False
oldgoal = Point()
oldgoal.x = 0
oldgoal.y = 0
waypoints = list()


class MoveBaseSquare():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.enable_limit = rospy.get_param(
            "~enable_mode", 1)  # 0:move_base ,1:p control
        self.distance_tolerance = rospy.get_param(
            "~distance_tolerance", 0.2)  # meters

        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)

        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)

        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 1.5)

        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)

        self.kp = rospy.get_param("~kp", 0.5)
        self.kpt = rospy.get_param("~kpi", 4)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo(
                    "Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 5 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(5))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        self.short_subscriber = rospy.Subscriber(
            'short_kinect', Bool, self.shortback)
        self.addgoal_pointsubscriber = rospy.Subscriber(
            '/addgoal_point', Point, self.imu_start, queue_size=5)
        rospy.loginfo("Subscribing to position...")

        # Wait for the  topic to become available
        rospy.wait_for_message('/addgoal_point', Point)
        pointmsg = Point()

        global i, k
        # waitting waypoint input and choose control method
        while i < k and not rospy.is_shutdown():
            if (shortok == True):
                # Intialize the waypoint goal
                goal = MoveBaseGoal()

                # Use the map frame to define goal poses
                goal.target_pose.header.frame_id = 'map'

                # Set the time stamp to "now"
                goal.target_pose.header.stamp = rospy.Time.now()

                # Set the goal pose to the i-th waypoint
                goal.target_pose.pose = waypoints[i]
                # print(i)
                # Start the robot moving toward the goal
                self.move(goal)

                i += 1

            elif (shortok == False):
                #i = k
                self.move_base.cancel_goal()
                break

    def imu_start(self, pointmsg):

        global k, waypoints, oldgoal
        #rospy.loginfo("receive goal(%d): %5.2f,%5.2f",k,pointmsg.x,pointmsg.y)

        if (oldgoal.x != pointmsg.x or oldgoal.y != pointmsg.y):
            k += 1
            waypoints.append(Pose(Point(pointmsg.x, pointmsg.y, 0.0), Quaternion(
                quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])))
            # print(waypoints,len(waypoints))
            rospy.loginfo("goal( %d ): %5.2f,%5.2f", k, pointmsg.x, pointmsg.y)

    def move(self, goal):
        # move_base
        if (self.enable_limit == 0):
            print("start move_base")
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)

            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(
                rospy.Duration(30))

            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")

        # Porportional Controller
        else:
            print("start Porportional Controller")
            global waypoints

            # Initialize the robot_position variable as a Point type
            move_cmd = Twist()
            robot_position = Point()
            self.goalpoint = Point()

            # Get the starting position values
            (robot_position, rotation) = self.get_odom()
            self.goalpoint.x = waypoints[i].position.x
            self.goalpoint.y = waypoints[i].position.y

            rospy.loginfo("goal[%5.3f, %5.3f],robot[%5.3f, %5.3f]", self.goalpoint.x,
                          self.goalpoint.y, robot_position.x, robot_position.y)

            
            self.distanceToGoal = sqrt(pow(
                (self.goalpoint.x - robot_position.x), 2) + pow((self.goalpoint.y - robot_position.y), 2))
            self.angleToGoal = (atan2(self.goalpoint.y - robot_position.y,
                                      self.goalpoint.x - robot_position.x) - rotation)

            # and not rospy.is_shutdown():
            while (self.distanceToGoal > self.distance_tolerance):

                if (shortok == True):

                    # Porportional Controller
                    # linear velocity in the x-axis: constant = 1.5
                    move_cmd.linear.x = self.kp * self.distanceToGoal  # + self.ki*I #+ self.kd*D
                    move_cmd.linear.x = copysign(max(self.min_linear_speed, min(
                        self.max_linear_speed, abs(move_cmd.linear.x))), move_cmd.linear.x)

                    # angular velocity in the z-axis: constant = 4
                    move_cmd.angular.z = self.kpt * self.angleToGoal
                    move_cmd.angular.z = copysign(max(self.min_angular_speed, min(
                        self.max_angular_speed, abs(move_cmd.angular.z))), move_cmd.angular.z)

                    # Publishing our move_cmd
                    self.cmd_vel_pub.publish(move_cmd)

                    # Publish at the desired rate.
                    rospy.sleep(0.5)

                    # Get the current position
                    (robot_position, rotation) = self.get_odom()
                    rospy.loginfo("ToGoal:[%5.3f,%5.2f], robot:[%5.3f,%5.3f]", self.distanceToGoal, degrees(
                        self.angleToGoal), robot_position.x, robot_position.y)

                    # Re-compute the distance,omega to the goal
                    self.distanceToGoal = sqrt(pow(
                        (self.goalpoint.x - robot_position.x), 2) + pow((self.goalpoint.y - robot_position.y), 2))
                    self.angleToGoal = (atan2(
                        self.goalpoint.y - robot_position.y, self.goalpoint.x - robot_position.x) - rotation)

                elif(shortok == False):
                    rospy.loginfo("distance < 4m")
                    break

            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            self.cmd_vel_pub.publish(move_cmd)

            # Stopping our robot after the movement is over
            rospy.loginfo(" Goal achieved(%5.3f,%5.3f),error:%5.3f",
                          robot_position.x, robot_position.y, self.distanceToGoal)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shortback(self, msg):
        global shortok
        shortok = msg.data

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(1)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        MoveBaseSquare()
        rospy.spin()  # If we press control + C, the node will stop.
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
