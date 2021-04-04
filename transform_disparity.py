from __future__ import print_function

from numpy.lib.function_base import median
import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from sensor_msgs.msg import CameraInfo
from stereo_msgs.msg import DisparityImage
import numpy as np


valid_window = RegionOfInterest(x_offset=70, y_offset=7, height=4294967281, width=4294967218, do_rectify=False)


def callback(msg):
    out = DisparityImage(
        header=msg.header, image=msg, f=415.69219381653056, T=0.05,
        min_disparity=0.0, max_disparity=63.0, delta_d=0.0625, valid_window=valid_window
    )
    print(1)
    pub.publish(out)

# def callback(msg):
#    print(msg.f, msg.T, msg.min_disparity, msg.max_disparity, msg.delta_d, msg.valid_window)


rospy.init_node('transform_disparity', anonymous=True)
rospy.Subscriber('/stereo_gray/dense_stereo/disparity/image', Image, callback)
# rospy.Subscriber('/disparity', DisparityImage, callback)
# rospy.Subscriber('/tesse/depth_cam/mono/image_raw', Image, callback)
pub = rospy.Publisher('/disparity', DisparityImage, queue_size=10)
rospy.spin()
