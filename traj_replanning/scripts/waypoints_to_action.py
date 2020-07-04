#! /usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations as tft
import math
import copy

from trackers_manager.srv import Transition
from std_trackers.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal

if __name__ == '__main__':

  rospy.init_node('traj_client')

  ns = '/ddk'

  client = actionlib.SimpleActionClient(ns+'/trackers_manager/trajectory_tracker/TrajectoryTracker', TrajectoryTrackerAction)
  rospy.loginfo("Waiting for server")
  client.wait_for_server()
  rospy.loginfo("Connected!")

  ps = Pose()
  ps.position.x = 0.5;
  ps.position.y = 0;
  ps.position.z = 1.0;

  q = tft.quaternion_from_euler(0.0, 0.0, 0.0)
  ps.orientation.x = q[0]
  ps.orientation.y = q[1]
  ps.orientation.z = q[2]
  ps.orientation.w = q[3]

  goal = TrajectoryTrackerGoal()
  goal.waypoints.append(copy.deepcopy(ps))
  ps.position.x = ps.position.x + 1.0
  #ps.position.y = ps.position.y + 0.5
  goal.waypoints.append(copy.deepcopy(ps))
  ps.position.x = ps.position.x + 1.0
  #ps.position.y = ps.position.y + 0.5
  goal.waypoints.append(copy.deepcopy(ps))

  print goal

  client.send_goal(goal)

  rospy.wait_for_service(ns+'/trackers_manager/transition')
  try:
    transition_tracker = rospy.ServiceProxy(ns+'/trackers_manager/transition', Transition)
    #transition_cmd = Transition();
    #transition_cmd.tracker = 'std_trackers/TrajectoryTracker';
    resp1 = transition_tracker('std_trackers/TrajectoryTracker')
    print resp1
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e