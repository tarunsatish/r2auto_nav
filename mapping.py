# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
# import cv2
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import scipy.stats
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# constants
rotatechange = 0.1
speedchange = 0.1
occ_bins = [-1, 0, 50, 100, 101]
stop_distance = 0.45
front_angle = 20
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata_S = np.array(msg.data)
        # occ_counts, edges, binnum = scipy.stats.binned_statistic(msgdata, np.nan, statistic='count', bins=occ_bins)
	    # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))
        # gridata=binnum-1
        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata_S + 1
        # reshape to 2D array using column order
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # self.occdata = np.uint8(gridata.reshape(msg.info.height,msg.info.width))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        # try:
        #     trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        # except (LookupException, ConnectivityException, ExtrapolationException) as e:
        #     self.get_logger().info('No transformation found')
        #     return
            
        # cur_pos = trans.transform.translation
        # cur_rot = trans.transform.rotation
        # # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # # convert quaternion to Euler angles
        # roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # # get map resolution
        # map_res = msg.info.resolution
        # # get map origin struct has fields of x, y, and z
        # map_origin = msg.info.origin.position
        # # get map grid positions for x, y position
        # grid_x = round((cur_pos.x - map_origin.x) / map_res)
        # grid_y = round(((cur_pos.y - map_origin.y) / map_res))
        # # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # self.occdata[grid_x][grid_y]=9

        # # print to file
        # np.savetxt(mapfile,np.around(self.occdata,decimals=0),fmt='%d')

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = 100


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle, mode):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        if mode==0:
            twist.linear.x = 0.0
        elif mode == 2:
            twist.linear.x = 0.01
        else:
            twist.linear.x = speedchange
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    # def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        # if self.laser_range.size != 0:
        #     # use nanargmax as there are nan's in laser_range added to replace 0's
        #     trace = self.laser_range[250]
        #     self.get_logger().info(str(laser_range[250]))
        #     if trace>0.35 or trace<0.01:
        #         self.rotatebot(350)
        #     elif trace<0.3 and trace>0.1:
        #         self.rotatebot(10)
        # else:
        #     self.pick_direction()

        # rotate to that direction
        # self.rotatebot(float(lr2i))

        # start moving
        # self.get_logger().info('Start moving')
        # # twist = Twist()
        # twist.linear.x = speedchange
        # twist.angular.z = 0.0
        # # not sure if this is really necessary, but things seem to work more
        # # reliably with this
        # time.sleep(1)
        # self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            # self.pick_direction()
            # flag=0
            # self.pick_direction()
            # self.pick_direction()
            # self.pick_direction()
            # self.pick_direction()

            while rclpy.ok():

                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()

                    # lri1 = (self.laser_range[range(250,290)]>float(0.5)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))
                    edge_f = self.laser_range[300]
                    edge_b = self.laser_range[200]

                    # while (flag==0):
                        # self.pick_direction()
                    #     self.get_logger().info('Start moving')
                    #     twist = Twist()
                    #     twist.linear.x = speedchange
                    #     twist.angular.z = 0.0
                    #     time.sleep(0.5)
                    #     self.publisher_.publish(twist)
                        # lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                        # if (len(lri[0])>0):
                            # self.stopbot()
                            # flag=1

                    # if the list is not empty  
                    if(len(lri[0])>0):
                            self.get_logger().info('stop and rotate left')
                            self.stopbot()
                            self.rotatebot(20,2)

                    # elif (len(lri1[0])>0):
                    #     self.get_logger().info('stop and rotate right')
                    #     self.stopbot()
                    #     self.rotatebot(355,0)

                    elif (0.01<edge_f<0.3 and edge_b>0.3) or (0.01<edge_f<0.3 and 0.01<edge_b<0.3):
                        self.get_logger().info('turn left')
                        self.stopbot()
                        self.rotatebot(1,1)
                    elif (0.01<edge_b<0.3 and edge_f>0.3) or (0.3<edge_f and 0.3<edge_b):
                        self.get_logger().info('turn right')
                        self.stopbot()
                        self.rotatebot(357,2)

                    else:
                        self.get_logger().info('Start moving')
                        twist = Twist()
                        twist.linear.x = speedchange
                        twist.angular.z = 0.0
                        time.sleep(0.5)
                        self.publisher_.publish(twist)
                    
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

#     def closure(self, mapdata):
#         # This function checks if mapdata contains a closed contour. The function
#         # assumes that the raw map data from SLAM has been modified so that
#         # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
#         # values go from 1 to 101.
#         # msgdata = np.array(msg.data)+1
#         # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
#         # closed contours have larger areas than arc length, while open contours have larger
#         # arc length than area. But in my experience, open contours can have areas larger than
#         # the arc length, but closed contours tend to have areas much larger than the arc length
#         # So, we will check for contour closure by checking if any of the contours
#         # have areas that are more than 10 times larger than the arc length
#         # This value may need to be adjusted with more testing.
#         ALTHRESH = 10
#         # We will slightly fill in the contours to make them easier to detect
#         DILATE_PIXELS = 3

#         # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
#         # and other positive values up to 101 (occupied)
#         # so we will apply a threshold of 2 to create a binary image with the
#         # occupied pixels set to 255 and everything else is set to 0
#         # we will use OpenCV's threshold function for this
#         ret,img2 = cv2.threshold(mapdata,2,255,0)
#         # we will perform some erosion and dilation to fill out the contours a
#         # little bit
#         element = cv2.getStructuringElement(cv2.MORPH_CROSS,(DILATE_PIXELS,DILATE_PIXELS))
#         img3 = cv2.erode(img2,element)
#         img4 = cv2.dilate(img2,element)
#         # use OpenCV's findContours function to identify contours
#         # OpenCV version 3 changed the number of return arguments, so we
#         # need to check the version of OpenCV installed so we know which argument
#         # to grab
#         fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#         (major, minor, _) = cv2.__version__.split(".")
#         if(major == '3'):
#             contours = fc[1]
#         else:
#             contours = fc[0]
#         # find number of contours returned
#         lc = len(contours)
#         # rospy.loginfo('# Contours: %s', str(lc))
#         # create array to compute ratio of area to arc length
#         cAL = np.zeros((lc,2))
#         for i in range(lc):
#             cAL[i,0] = cv2.contourArea(contours[i])
#             cAL[i,1] = cv2.arcLength(contours[i], True)

#         # closed contours tend to have a much higher area to arc length ratio,
#         # so if there are no contours with high ratios, we can safely say
#         # there are no closed contours
#         cALratio = cAL[:,0]/cAL[:,1]
#         # rospy.loginfo('Closure: %s', str(cALratio))
#         if np.any(cALratio > ALTHRESH):
#             return True
#         else:
#             return False

def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
