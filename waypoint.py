#!/usr/bin/env python
import rospy
import os
import tf
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
#imports

# Use absolute path and expand the user's home directory
FILEPATH = os.path.expanduser('~/rallycar_ws/src/maxROStappen/src/scripts/knoy_FinalMap.txt')

#waypoint object
class waypoint(object):
    #init method
    def __init__(self):
        self.marker_id = 1
        rospy.init_node('echoer')
        # subscribe to "/move_base_simple/goal" to get picked waypoints using 2D Nav Goal in RViz
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_way_point)
        # display picked waypoints and path between waypoints in RViz
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.pose_arr = []
        self.points_marker = Marker()
        self.path_marker = Marker()
        self.init_markers()

    def init_markers(self):
        # Initialize points marker
        self.points_marker.header.frame_id = "map"
        self.points_marker.type = Marker.POINTS
        self.points_marker.action = Marker.ADD
        self.points_marker.lifetime = rospy.Duration(0)
        self.points_marker.scale.x = 0.2  # Increased scale for visibility
        self.points_marker.scale.y = 0.2  # Increased scale for visibility
        self.points_marker.color.a = 1.0
        self.points_marker.color.r = 1.0
        self.points_marker.color.g = 0.0
        self.points_marker.color.b = 0.0
        self.points_marker.pose.orientation.w = 1.0
        self.points_marker.id = 0

        # Initialize path marker
        self.path_marker.header.frame_id = "map"
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD
        self.path_marker.lifetime = rospy.Duration(0)
        self.path_marker.scale.x = 0.1  # Width of the line
        self.path_marker.color.a = 1.0
        self.path_marker.color.r = 0.0
        self.path_marker.color.g = 1.0  # Set color to green for better visibility
        self.path_marker.color.b = 0.0
        self.path_marker.pose.orientation.w = 1.0
        self.path_marker.id = 1

    def get_way_point(self, msg):
        # get orientation and convert quaternion to euler (roll,waypoints_speedway2 pitch, yaw)
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(euler[2])
        rospy.loginfo("X: {}, Y: {}, Yaw: {} degrees".format(msg.pose.position.x, msg.pose.position.y, yaw))
        
        # Append to pose array
        point = Point(x=msg.pose.position.x, y=msg.pose.position.y)
        self.pose_arr.append([msg.pose.position.x, msg.pose.position.y, yaw])

        # Append point to both markers
        self.points_marker.points.append(point)
        self.path_marker.points.append(point)

        # Publish the updated markers
        marker_array = MarkerArray(markers=[self.points_marker, self.path_marker])
        self.publisher.publish(marker_array)

        # Save the waypoints each time a new one is added
        self.save()

    def save(self, fname=FILEPATH):  # Default parameter set to FILEPATH
        with open(fname, 'w') as f:
            for p in self.pose_arr:
                f.write("%s\n" % p)
        rospy.loginfo("Waypoints saved to {}".format(fname))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    print("*********** waypoint.py: read and display way point on the map ***********")
    waypoint().run()

