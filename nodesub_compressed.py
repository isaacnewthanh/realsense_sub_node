#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import json
import numpy as np
import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import matplotlib.pyplot as plt

br = CvBridge()

def jsonw(msg,a):
    dictionary = msg
    json_object = json.dumps(dictionary, indent = 4)
    with open(a, "w") as outfile:
        outfile.write(json_object)

def callback(msg):
    str_msg = msg.data
    buf = np.ndarray(shape=(1,len(str_msg)), dtype=np.uint8, buffer=msg.data)
    img = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
    rospy.loginfo("d: %s",img)
    # dict = {
    #     "img": ls
    # }
    # jsonw(ls,"sample2.json")
    
    # cv2.imwrite(a,img)
    return img
def raw(msg):
    img = callback(msg)
    cv2.imwrite("raw.png",img)

def infra(msg):
    img = callback(msg)
    cv2.imwrite("infra.png",img)

def depth(msg):
    img = callback(msg)
    cv2.imwrite("depth.png",img)
    dist = rs.get_distance(rs.img,1,1)
    print(dist)


def listener():

    
    rospy.init_node('nodesub_compressed', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage , raw)
    rospy.Subscriber("/camera/infra1/image_raw/compressed", CompressedImage , infra)
    rospy.Subscriber("/camera/depth/image_raw/compressed", CompressedImage , depth)

    rospy.spin()

if __name__ == '__main__':
    listener()
