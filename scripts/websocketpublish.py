#!/usr/bin/env python

import rospy
from websocket import create_connection
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from limelight_ros.msg import LimelightStamped
from math import radians

uri = "ws://localhost:5805"
pub = rospy.Publisher('chatter', LimelightStamped, queue_size=10)
rospy.init_node('talker', anonymous=True)



if __name__ == '__main__':
    try:
        ws = create_connection(uri)
        #print "Running..."
        while not rospy.is_shutdown():
            wsstring = ws.recv()
            #print wsstring
            stringparts = wsstring.split(' ')
            assert len(stringparts) ==2
            assert stringparts[0] == "status_update"
            data = stringparts[1].split(':')
            msg = LimelightStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()

            tx = float(data[0])
            ty = float(data[1])
            rot = float(data[3])
            #print tx, ty, rot
            tf_quat = quaternion_from_euler(radians(tx), radians(ty), radians(rot), 'rxyz')
            msg.targetLocation = Quaternion(tf_quat[0], tf_quat[1], tf_quat[2], tf_quat[3])

            #Convert from percent
            msg.targetArea.data = float(data[2])/100.

            msg.hasTargets.data = bool(int(data[5]))

            msg.pipeline.data = int(data[6])

            pub.publish(msg)
    except:
        ws.close()


