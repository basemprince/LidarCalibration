import cv2
#import pypcd
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import rospy
import os
import subprocess, datetime, time, signal
from subprocess import STDOUT,check_output

class Photographer:
    def __init__(self, rate=10):
        rospy.init_node('listener')
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber('/zed/zed_node/left_raw/image_raw_color', Image, self._callback_image)
        #self.sub_cloud = rospy.Subscriber('something', PointCloud2, self._callback_cloud)
        self.cv_image = None
        self.rate = rospy.Rate(rate)
       

    def _callback_image(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #cv2.imshow("image", self.cv_image)
        #cv2.waitKey(1)
    
    def _callback_cloud(self, data):
        pass

    def take_photo(self, filename):
        cv2.imwrite(filename, self.cv_image)


def timeout_command(command, timeout):
  """call shell-command and either return its output or kill it
  if it doesn't normally exit within timeout seconds and return None"""
  
  start = datetime.datetime.now()
  process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,shell=True)
  while process.poll() is None:
    time.sleep(0.1)
    now = datetime.datetime.now()
    if (now - start).seconds> timeout:
      os.kill(process.pid, signal.SIGKILL)
      os.waitpid(-1, os.WNOHANG)
      return None
  return process.stdout.read()



def main():
    ph = Photographer()
    iter = 1
    command = 'rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_points _prefix:=/home/driverless/Documents/MATLAB/Examples/R2020b/lidar/LidarCameraCalibrationExample/'
    stop_command = 'rosnode kill pcl_ros'

    iter += 1
    ph.rate.sleep()

    while raw_input("Take photo? (y/n)") == "y":
        ph.take_photo(str(iter)+'.png')
        #p = subprocess.Popen(command, shell=True)

        time.sleep(3)

        #subprocess.Popen(stop_command, shell=True)

        #output = check_output("rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_points _prefix:=/home/driverless/Documents/MATLAB/Examples/R2020b/lidar/LidarCameraCalibrationExample/", timeout=3)
        #output = timeout_command(["rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_points _prefix:=/home/driverless/Documents/MATLAB/Examples/R2020b/lidar/LidarCameraCalibrationExample/"],2)
        #script = os.subprocess('rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_points _prefix:=/home/driverless/Documents/MATLAB/Examples/R2020b/lidar/LidarCameraCalibrationExample/'+str(iter))        #%s' %str(iter))
        script = os.system('rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_points _prefix:=/home/driverless/Documents/MATLAB/Examples/R2020b/lidar/LidarCameraCalibrationExample/'+str(iter))
        
        iter += 1
        ph.rate.sleep()


if __name__ == "__main__":
    main()