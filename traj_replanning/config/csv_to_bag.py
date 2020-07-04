import csv

import rospy
import rosbag
import tf
from std_msgs.msg import Int32, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

file_name = 'wp_X100B_to_CRD_rail_optimal_dark'

bag = rosbag.Bag(file_name + '.bag', 'w')

frame_id = 'world'

path = Path()
with open(file_name + '.csv', 'rb') as csvfile:
  spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')

  for row in spamreader:
    print ', '.join(row)
    print len(row)

    if len(row) < 4:
      print 'WARN: Yaw not set in CSV, setting to 0'
      row.append(0)

    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(row[0])
    ps.pose.position.y = float(row[1])
    ps.pose.position.z = float(row[2])

    quaternion = tf.transformations.quaternion_from_euler(0, 0, float(row[3]))

    #type(pose) = geometry_msgs.msg.Pose
    ps.pose.orientation.x = quaternion[0]
    ps.pose.orientation.y = quaternion[1]
    ps.pose.orientation.z = quaternion[2]
    ps.pose.orientation.w = quaternion[3]

    path.poses.append(ps)

    print ps

try:
  bag.write('waypoints', path)
finally:
  bag.close()