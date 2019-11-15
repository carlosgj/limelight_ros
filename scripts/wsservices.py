#!/usr/bin/env python

import rospy
from websocket import create_connection
from limelight_ros.srv import LED, LEDResponse

wsuri = "ws://localhost:5805"

def handle_LED_command(req):
    #print "Setting state to %d"%(req.state)
    ws = create_connection(wsuri)
    ws.send("set_setting 0*pipeline_led_enabled:%d#"%req.state)
    ws.close()
    return LEDResponse()

def server():
    rospy.init_node('Limelight_LED_server')
    s = rospy.Service('LED', LED, handle_LED_command)
    rospy.spin()

if __name__ == "__main__":
    server()
