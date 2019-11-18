#!/usr/bin/env python

import rospy
from websocket import create_connection
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Quaternion, Transform
from tf.transformations import quaternion_from_euler
from limelight_ros.msg import LimelightStamped
from math import radians

wsuri = "ws://localhost:5805"
pub = rospy.Publisher('limelightData', LimelightStamped, queue_size=10)
rospy.init_node('limelight', anonymous=True)



if __name__ == '__main__':
    try:
        ws = create_connection(wsuri)
        #print "Running..."
        while not rospy.is_shutdown():
            wsstring = ws.recv()
            #print wsstring
            stringparts = wsstring.split(' ')
            assert len(stringparts) ==2
            if stringparts[0] == "status_update":
                data = stringparts[1].split(':')
                
                pipelineLatency = float(data[4])
                
                msg = LimelightStamped()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
    
                tx = float(data[0])
                ty = float(data[1])
                ts = float(data[3])
                #print tx, ty, rot
                tf_quat = quaternion_from_euler(radians(tx), radians(ty), radians(ts), 'rxyz')
                msg.targetLocation = Quaternion(tf_quat[0], tf_quat[1], tf_quat[2], tf_quat[3])
    
                #Convert from percent
                msg.targetArea.data = float(data[2])/100.
    
                msg.hasTargets.data = bool(int(data[5]))
    
                msg.pipeline.data = int(data[6])
                
                #Don't know what this stuff is
                ignoringNetworkTables = bool(int(data[7]))
                imageCount = int(data[8])
                interfaceNeedsRefresh = bool(int(data[9]))
                usingGrip = bool(int(data[10]))
                pipelineImageCount = int(data[11])
                snapshotMode = int(data[12])
                hwType = int(data[13])
                
                posetX = float(data[14])
                posetY = float(data[15])
                posetZ = float(data[16])
                poserX = float(data[17])
                poserY = float(data[18])
                poserZ = float(data[19])
                tf_quat = quaternion_from_euler(radians(poserX), radians(poserY), radians(poserZ), 'rxyz')
                msg.cameraPose.translation.x = posetX
                msg.cameraPose.translation.y = posetY
                msg.cameraPose.translation.z = posetZ
                msg.cameraPose.rotation = Quaternion(tf_quat[0], tf_quat[1], tf_quat[2], tf_quat[3])
                
                pub.publish(msg)
            elif stringparts[0] == 'flushed_settings':
                #Limelight is just saying that it flushed settings to disk, don't need to do anything here
                pass
            else:
                print stringparts
    except:
        ws.close()
	raise

