#! /usr/bin/python

import rospy
from rospy import loginfo as INFO
from rospy import logerr as ERR
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [[4, -21.5], [5.5, 20], [-10, 18], [-4, -21.5]]


def main():
    INFO("Connecting to move_base")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    INFO("Connected")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.orientation.w = 1.0

    for waypoint in waypoints:
        INFO("Going to %s." % waypoint)

        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = waypoint

        client.send_goal(goal)
        # if not (wait := client.wait_for_result()):
        wait = client.wait_for_result()
        if not wait:
            INFO("FAIL")
            rospy.signal_shutdown("Goal not accomplished")

        INFO("Reached.")


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_sender')
        main()
    except rospy.ROSInterruptException:
        ERR("FAILED")
