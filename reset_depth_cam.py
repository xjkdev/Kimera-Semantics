from __future__ import print_function

from numpy.lib.function_base import median
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np

l_cache = None
r_cache = None

def callbackl(msg):
    global l_cache
    if not l_cache:
        P = list(msg.P)
        P[3] = 0.05 * 415.69219381653056
        l_cache = tuple(P)
    msg.P = l_cache
    publ.publish(msg)

def callbackr(msg):
    global r_cache
    if not r_cache:
        P = list(msg.P)
        P[3] = -0.05 * 415.69219381653056
        r_cache = tuple(P)
    msg.P = r_cache
    pubr.publish(msg)



rospy.init_node('reset_depth_cam', anonymous=True)
# rospy.Subscriber('/elas/depth', Image, callback)
# rospy.Subscriber('/tesse/depth_cam/mono/image_raw', Image, callback)
rospy.Subscriber('/cam0/camera_info', CameraInfo, callbackl)
rospy.Subscriber('/cam1/camera_info', CameraInfo, callbackr)
publ = rospy.Publisher('/cam01/camera_info', CameraInfo, queue_size=10)
pubr = rospy.Publisher('/cam11/camera_info', CameraInfo, queue_size=10)
rospy.spin()
