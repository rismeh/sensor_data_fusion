#!/usr/bin/env python

# Sensor Data Fusion: Occupancy gridmap
# Author: Lucila Patino Studencki
# 
# Test gridmap

import rclpy
from rclpy.node import Node

# import gridmap class
from sensor_data_fusion.gridmap_hsc import Gridmap

# import laserscan sample
from sensor_msgs.msg import LaserScan
from sensor_data_fusion.laserscan_sample import get_laserscan_sample

import numpy as np

class Occupancy_Gridmap_Laser(Node):

    def __init__(self, size, resolution, topic, z_max=10, alpha=1, \
        beta = 10.*np.pi/180.):

        super().__init__('my_g_map_laser')

        # Initialize the gridmap
        self.gridmap = Gridmap(size, resolution, topic, 0.5, 0.1, 0.2)

	    # convert z_max to m
        self.z_max = z_max / self.gridmap.resolution
        # set increment of cell and angle 
        self.alpha = alpha
        self.beta = beta

        # create timer for callbacking publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_map)   

        # TODO: register subscriber to topic "/scan"   if rosbag is used: '/dolly/laser_scan'
        self.subscription = self.create_subscription(LaserScan, "/dolly/laser_scan", self.callbackLaserScan, 10)
        self.subscription  # prevent unused variable warning

    # Inverse sensor (laserscanner) model
    # Inputs:
    # 	x_i: coordinates of gridcell center
    # 	x_k: vehicle pose at time k (x,y,heading)
    # 	z_k: vector of laser-distance measurements at time k
    # Outputs:
    #	logOdds: indicating occupied cell, free cell

    def inverse_range_sensor_model(self, x_i, x_k, z_k):

        # measurement vector in polar-coordinates from scan
        z_meas = z_k[:, 0]
        phi_meas = z_k[:, 1]

        # calculate distance and angle to each cell 
        r = np.linalg.norm(x_i - x_k[0:2])
        phi = np.arctan2(x_i[1]-x_k[1], x_i[0]-x_k[0]) - x_k[2]

        # search for beam closest to measured angle
        d_phi = np.abs(phi - phi_meas)
    	# consider only angles between -pi and pi
        d_phi[d_phi > np.pi] -= 2. * np.pi
        k = np.argmin(np.abs(d_phi))
        # measure range and angle of closest angle
        z_k_beam = z_meas[k]
        phi_k = phi_meas[k]

        # Unobservable, distance to cell bigger than current range
        # TODO: implement this funtion, which logOdd value shall be returned?
        if(r > self.z_max) or (r > z_k_beam + (self.alpha / 2)) or (abs(phi - phi_k) > (self.beta/2)):
            return(self.gridmap.l_0)

        # Occupied cell, range inside of cell   
        # TODO: implement this funtion, which logOdd value shall be returned?
        elif(z_k_beam < self.z_max) and (abs(r-z_k_beam) < self.alpha / 2):
            return(self.gridmap.l_occ)

        # Free cell, Distance to cell smaller than range
        # TODO: implement this funtion, which logOdd value shall be returned?
        elif(r<z_k_beam):
            return(self.gridmap.l_free)
    
        self.get_logger().warn('laser_model: undefined case')
        return self.gridmap.l_0
    
    # Check if cell in pereptual field
    # Inverse sensor (laserscanner) model
    # Inputs:
    # 	x_i: coordinates of gridcell center
    # 	x_k: vehicle pose at time k (x,y,heading)
    # 	z_k: vector of laser-distance measurements at time k
    # Outputs:
    #	False: cell not in perceptual field
    #	True: cell in perceptual field

    def in_perceptual_field(self, x_i, x_k, z_k):

        # Check distance
        r = np.linalg.norm(x_i - x_k[0:2])
        # TODO: if not in Range -> false
        if(r > self.z_max):
            return(False)

        # Check angle
        # TODO: calculate the angle between vehicle and cell
        phi = np.arctan2(x_i[1]-x_k[1], x_i[0]-x_k[0]) - x_k[2]

        # check current angle for min and max phi including cell width 
        phi_min = z_k[0, 1] - self.beta/2.
        phi_max = z_k[-1, 1] + self.beta/2.
        if ((phi-phi_min) % (2. * np.pi) <= (phi_max-phi_min) % (2. * np.pi)):
            return(True)
        else:
            return(False)

    # Iterate over all cells and get probabilities
    # Inputs:
    # 	x_k: vehicle pose at time k (x,y,heading)
    # 	z_k: vector of laser-distance measurements at time k
    def build_gridmap(self, x_k, z_k):

        # for all cells (cell_x, cell_y) do
        for cell_x in range(self.gridmap.size):
            for cell_y in range(self.gridmap.size):
                x_i = np.array([cell_x, cell_y]) + 0.5

                # if cell in perceptual field of vehicle get log odds
                if self.in_perceptual_field(x_i, x_k, z_k):
                    l_curr = self.inverse_range_sensor_model \
                        (x_i, x_k, z_k) 
                    self.gridmap.update_logOdds(cell_x, cell_y, l_curr)
 
        
    def preprocessing_measurements(self, pose_m, scan_m, z_max_m, beta):
        # Convert scan from meters to grid units
        scan = np.array([ \
            scan_m[:, 0] / self.gridmap.resolution,  # range
            scan_m[:, 1]]).T                         # angle

        # Limit infinite beams to z_max_grid
        scan[np.isnan(scan[:, 0]), 0] = self.z_max

        # Convert from meters to grid units and store
        self.z_max = z_max_m / self.gridmap.resolution
        self.beta = beta

        # Convert pose from meters to grid units
        pose = pose_m * 1.
        pose[:2] /= self.gridmap.resolution
        pose[:2] += self.gridmap.size / 2.
        pose[:1] -= self.gridmap.size / 2.

        return(scan, pose)

    def callbackLaserScan(self, scan):
        N = len(scan.ranges)
        scan_data = np.zeros((N, 2))
        scan_data[:, 0] = np.array(scan.ranges)
        scan_data[:, 1] = scan.angle_min + np.arange(N) * scan.angle_increment

    	# TODO: assign z_max according to 'LaserScan.range_max' 
        z_max = scan.range_max

    	# TODO: assign beta according to 'LaserScan.angle_increment'
        beta = scan.angle_increment


    	# Init pose (x,y,heading)
        pose = np.zeros(3) # corresponding topic needs to be subscribed when using real data

    	# prepare measurements
        scan_prep, pose_prep = self.preprocessing_measurements(pose, scan_data, z_max, beta)

    	# call mapping method on m (instance of 'Gridmap_Mapping')
        self.build_gridmap(pose_prep, scan_prep)

    def publish_map(self):
        self.gridmap.publish_gridmap()

def mapping_with_sample(args = None):

    rclpy.init(args=args)
    node = rclpy.create_node('Occupancy_Grid_Node')

    # Load laserscan sample
    scan = get_laserscan_sample()

    # Init pose
    pose = np.zeros(3)

    N = len(scan.ranges)
    # create array of measured points in polar coordinates
    # first line ranges, second line angles
    scan_data = np.zeros((N, 2))
    scan_data[:,0] = scan.ranges

    scan_data[0,1] = scan.angle_min
    for i in range(1, N):
        scan_data[i,1] = scan_data[i-1,1] + scan.angle_increment
    # maximal range according to measurement
    z_max = scan.range_max
    # assign alpha and beta according to
    # 'LaserScan.angle_increment'
    alpha = 1. # since calculations are given in "cell"-units it is one
    beta = scan.angle_increment

    # Create instance of Occupancy_Gridmap_Laser on topic hsc_map
    my_map = Occupancy_Gridmap_Laser(60, 0.2, "hsc_map", z_max, alpha, beta)

    # prepare measurements
    scan_prep, pose_prep = my_map.preprocessing_measurements(pose, scan_data, z_max, beta)
    # Call grid mapping algorithm
    my_map.build_gridmap(pose_prep, scan_prep)

    # Set ROS Node
    rclpy.spin(my_map)

    my_map.destroy_node()
    rclpy.shutdown()

def mapping_with_laser_scan(args = None):

    rclpy.init(args=args)
    node = rclpy.create_node('Occupancy_Grid_Dynamic_Node')

    # Create instance of Gridmap_Mapping on topic my_map
    my_map = Occupancy_Gridmap_Laser(60, 0.2, "hsc_map")

    # Set ROS Node
    rclpy.spin(my_map)

    my_map.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__mapping_with_sample__':
    mapping_with_sample()          # Map from saved sample

if __name__ == '__mapping_with_laser_scan__':
    mapping_with_laser_scan()       # Laser Scan with ROS Bag
