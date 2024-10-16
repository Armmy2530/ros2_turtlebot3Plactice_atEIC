#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import math

class navRobot(Node):
    def __init__(self):
        super().__init__('nav2_cmd_node')
        self.get_logger().info('Waypoint started')
        self.nav = BasicNavigator()

        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is activated')

        self.initial = PoseStamped()
        self.set_initial_pose()

        self.wp = []
        self.legoRoom_StartDock = (-0.6875389218330383,
                                   0.3110027611255646, 0.0)
        self.legoRoom_place1 = (2.040781021118164, 1.5769683122634888, 0.0)
        self.legoRoom_place2 = (1.357548713684082, -0.7587277889251709, 0.0)
        self.legoRoom_place3 = (-4.283427715301514, -1.6083461046218872, 0.0)
        self.legoRoom_place4 = (0.21580882370471954, 0.537469208240509, 0.0)

        self.StartDock = (-0.8038409352302551, -0.3733225166797638, 0.0)
        self.B7pillar1 = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B7pillar2 = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B7pillar3 = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B7stage = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B7B6connecter = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B6antroom = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B6atm = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B6cafe = (2.9363372325897217, 0.9414265751838684, 0.0)
        self.B6ladder = (2.9363372325897217, 0.9414265751838684, 0.0)

    def set_initial_pose(self,x=-0.035,y=1.577,yaw=0.0):
        self.initial.header.frame_id = "map"
        self.initial.header.stamp = self.get_clock().now().to_msg()
        self.initial.pose.position.x = -x
        self.initial.pose.position.y = y

        q = self.quaternion_from_euler(0, 0, yaw)

        self.initial.pose.orientation.w = q[0]
        self.initial.pose.orientation.x = q[1]
        self.initial.pose.orientation.y = q[2]
        self.initial.pose.orientation.z = q[3]

        self.nav.setInitialPose(self.initial)

    def set_point(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        q = self.quaternion_from_euler(0, 0, theta)

        goal_pose.pose.orientation.w = q[0]
        goal_pose.pose.orientation.x = q[1]
        goal_pose.pose.orientation.y = q[2]
        goal_pose.pose.orientation.z = q[3]

        return goal_pose

    def goto(self, x, y, theta):
        target = self.set_point(x, y, theta)
        self.nav.goToPose(target)
        i=0
        while not self.nav.isTaskComplete():
            i = i + 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

    def waypoint(self):
        pose = self.set_point(self.legoRoom_place1(
            0), self.legoRoom_place1(1), self.legoRoom_place1(2))
        self.wp.append(pose)

        pose = self.set_point(self.legoRoom_place2(
            0), self.legoRoom_place2(1), self.legoRoom_place2(2))
        self.wp.append(pose)

        pose = self.set_point(self.legoRoom_place3(
            0), self.legoRoom_place1(1), self.legoRoom_place3(2))
        self.wp.append(pose)

        pose = self.set_point(self.legoRoom_place4(
            0), self.legoRoom_place4(1), self.legoRoom_place4(2))
        self.wp.append(pose)

        pose = self.set_point(self.legoRoom_place1(
            0), self.legoRoom_place1(1), self.legoRoom_place1(2))
        self.wp.append(pose)

        self.nav.followWaypoints(self.wp)

        while not self.nav.isNavComplete():
            print(self.nav.isNavComplete())
            feedback = self.nav.getFeedback()
            print(feedback)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        q = [0]*4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


def main():
    rclpy.init()
    nr = navRobot()
    nr.goto(3.80,-0.03,0)
    # nr.waypoint()
    nr.destroy_node()


if __name__ == "__main__":
    main()