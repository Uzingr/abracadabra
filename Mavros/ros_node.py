#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Mavlink

def vicon_callback(data):
    mavlink_msg = Mavlink()
    mavlink_msg.sysid = 1
    mavlink_msg.compid = 1
    mavlink_msg.msgid = 113  # ID for MAVLink HIL_GPS message

    # HIL_GPS message fields: 
    # time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible

    # Convert position data from Vicon to HIL_GPS format
    lat = int(data.pose.position.x * 1e7)  # Convert to degrees * 1e7
    lon = int(data.pose.position.y * 1e7)  # Convert to degrees * 1e7
    alt = int(data.pose.position.z * 1e3)  # Convert to millimeters

    # Example values for other fields (you may need to adjust or calculate these)
    time_usec = int(rospy.Time.now().to_nsec() / 1000)
    fix_type = 3  # Assuming 3D fix
    eph = 100
    epv = 100
    vel = 0
    vn = 0
    ve = 0
    vd = 0
    cog = 0
    satellites_visible = 10

    # Populate the MAVLink payload (simplified example, adjust as needed)
    mavlink_msg.payload64 = [
        time_usec, fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible
    ]

    mavlink_pub.publish(mavlink_msg)

if __name__ == '__main__':
    rospy.init_node('vicon_to_mavlink')
    mavlink_pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=10)
    rospy.Subscriber('/vicon/pose', PoseStamped, vicon_callback)
    rospy.spin()
