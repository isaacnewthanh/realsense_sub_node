import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2 as sensor_msgs_pc2
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs.msg 
import pyrealsense2 as rs
import cv2
import numpy as np
from PIL import Image
i=0
class CameraFeed(object):

    def __init__(self):
        
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw", sensor_msgs.msg.Image, self.callback)
        self.br = CvBridge()

    def callback(self, data):
        try:
            img = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        global i
        depth_array = np.array(img, dtype=np.float32)
        # im = Image.fromarray(depth_array)
        # im = im.convert('L')
        # idx = str(i).zfill(4)
        # im.save('/home/isaacnewthanh/isaacnewthanh/realsense_ws/frame{index}.png'.format(index = idx))
        # i+=1
        rospy.loginfo('depth_idx: %s', depth_array[1:2])
        cv2.imwrite("data.png",img)
def main():
    print('Running ...')
    cam = CameraFeed()
    rospy.init_node("cam", anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main()