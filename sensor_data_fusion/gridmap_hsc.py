#!/usr/bin/env python3

# Sensor Data Fusion: Occupancy gridmap
# Authors: Lucila Patino Studencki / Nitin Saravana Kannan 
# 
# Gridmap Node

import rclpy
from rclpy.node import Node

import numpy as np

import transforms3d                             # install this module:   pip install transforms3d
from transforms3d.euler import euler2quat

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist 

import tf2_py
import tf2_ros

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class Gridmap(Node):
       
    def __init__(self, size, resolution, topic_name, p_0, p_occ, p_free):
        super().__init__('my_g_map')
        print('Gridmap Node Initialized, It will publish an OccupancyGrid Message as topic:', topic_name)
    # Initialize grid map parameters
        self.size = size
        self.resolution = resolution

	# Calculate default log odds
        self.l_0 = self.prob2logOdds(p_0)
        self.l_occ = self.prob2logOdds(p_occ)
        self.l_free = self.prob2logOdds(p_free)
        self.log_odds = self.l_0 * np.ones((size, size))

    # Prepare Publisher
        self.topic_name = topic_name
        self.pub = self.create_publisher(OccupancyGrid, topic_name, 1)

    # create transformBroadcaster instance
        self.br = tf2_ros.TransformBroadcaster(self)

    # create timer for callbacking publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_gridmap)                     

    # Transformation from probability to logOdds 
    def prob2logOdds(self, prob):
        li_k = np.log(prob/(1-prob))
        # todo: calculation
        return(li_k)  
    
    # Transformation from logOdds to probabilities
    def logOdds2prob(self, log_o):
        pi_k = 1 - 1/ (1+ np.exp(log_o))
        # todo: calculation
        return(pi_k)

    #def update_cell(self, cell_x, cell_y, l_i):
    def update_logOdds(self, cell_x, cell_y, l_mi):
        self.log_odds[cell_y, cell_x] +=  l_mi

        # todo: calculation

    def publish_gridmap(self):

    # Filling OccupancyGrid Message for the topic
        msg = OccupancyGrid()
        msg.header.frame_id = self.topic_name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.size
        msg.info.height = self.size

        # Transform log_odds to probabilites and flatten array
        msg_data = self.logOdds2prob(self.log_odds).flatten()

    # Normalize and set to [0, 100]
        if (np.max(msg_data) != np.min(msg_data)) :
            msg_data -= np.min(msg_data)
            msg_data *= 100. / np.max(msg_data)
        else:
            msg_data *= 100 

        # assign -1 to nan-values
        msg_data[np.isnan(msg_data)] = -1

        # cast to int8
        msg.data = msg_data.astype(np.int8).tolist()

    # transformation with respect to /map for visualization in RViz
	    # grid center (offset)
        cx = self.size * self.resolution / 2.
        cy = self.size * self.resolution / 2.
        # orientation
        quat = euler2quat(0., 0., 0.)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"                  
        tf_msg.child_frame_id = self.topic_name  
        tf_msg.transform.translation.x = cx
        tf_msg.transform.translation.y = cy
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

    # Send coordinate transformation
        self.br.sendTransform(tf_msg)

    # Publishing grid message
        self.pub.publish(msg)

# Testing the module
def test_gridmap_node_checkered(args = None):   

    rclpy.init(args=args)

    # Generate Gridmap object
    g_map = Gridmap(20, 0.2, "hsc_map", 0.5, 0.8, 0.2)

    # Prepare Checkered Shape
    
    g_map.log_odds[1::2, :] = g_map.l_free
    g_map.log_odds[:, 1::2] = g_map.l_free

    #g_map.log_odds[0:20:19,0:20:19] = g_map.l_free
    #g_map.log_odds[1::17,1::17] = g_map.l_free
    #g_map.log_odds[2::15,2::15] = g_map.l_free
    #g_map.log_odds[3::13,3::13] = g_map.l_free
    #g_map.log_odds[4::11,4::11] = g_map.l_free
    #g_map.log_odds[5::9,5::9] = g_map.l_free
    #g_map.log_odds[6::7,6::7] = g_map.l_free
    #g_map.log_odds[7:13:5,7:13:5] = g_map.l_free
    #g_map.log_odds[8:12:3,8:12:3] = g_map.l_free
    #g_map.log_odds[9:11:,9:11:] = g_map.l_free
    #g_map.initialized = True

    # Set ROS Node
    rclpy.spin(g_map)

    g_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__test_gridmap_node_checkered__':
    test_gridmap_node_checkered()
