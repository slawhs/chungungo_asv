#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from sklearn.cluster import DBSCAN, OPTICS

import numpy as np

# ------ LiDAR Parameters ------
TIME_INCREMENT = 0.00011
ANGLE_INCREMENT = 0.005806980188935995  #? radians
ANGLE_MIN = -3.1241390705108643
ANGLE_MAX = 3.1415927410125732
RANGE_MIN = 0.15
RANGE_MAX = 12.0
N_SAMPLES = 720

# ------ Clustering parameters ------
EPS = 0.05  #? Distance (meters) between two points to be considered in the same cluster
CLUSTER_MIN_SAMPLES = 10

MIN_DIST_FILTER = 0.3
MAX_DIST_FILTER = 2.0
    
class Lidar(Node): 
    def __init__(self):
        super().__init__("Lidar")

        # -------- Publishers and Subscribers --------
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.lidar_scan_cb, 1)
        self.laser_pub = self.create_publisher(String, "/object_ahead", 1)
        self.filtered_scan_pub = self.create_publisher(LaserScan, "/filtered_scan", 1)
        self.centroids_pub = self.create_publisher(LaserScan, "/centroids", 1)
        
        # -------- Atributes --------
        self.laser_ranges = None
        self.polar_samples = None
        self.carthesian_samples = None
        self.clustering = OPTICS(eps=EPS, min_samples=CLUSTER_MIN_SAMPLES)

    def lidar_scan_cb(self, msg: LaserScan):  #? Jorge: 720 muestras - Nosotros 1080 muestras
        # self.front_ranges = msg.ranges[1035:] + msg.ranges[:45]  #? 30°
        self.detect_buoys(msg)
        # self.publish_front_scan(msg)
        # if min(self.front_ranges) < 0.8:
        #     # self.get_logger().info(f"Object Ahead!")
        #     string_msg = String()
        #     string_msg.data = "Object Ahead!"
        #     self.laser_pub.publish(string_msg)

    def publish_front_scan(self, msg):
        msg.ranges = self.front_ranges
        self.front_laser_pub.publish(msg)

    def detect_buoys(self, msg):
        self.samples_to_polar(msg)
        self.filter_samples()
        self.publish_samples()
        # self.polar_to_carthesian()

        # if self.carthesian_samples.shape[0] != 0:
        #     clusters = self.clustering.fit(self.carthesian_samples)
        #     centroids = self.get_centroids(clusters)
        #     self.publish_centroids(centroids)

    # def filter_samples(self):
    #     np.nan_to_num(self.polar_samples, copy=False, posinf=13.0)
    #     distance_mask = (self.polar_samples[:, 0] > 0.3) & (self.polar_samples[:, 0] < 2.0)  # Filter close objects
    #     angle_mask = (self.polar_samples[:, 1] >= 0) & (self.polar_samples[:, 1] <= np.pi)  # Filter only front objects
    #     self.polar_samples = self.polar_samples[distance_mask & angle_mask]  

    def wrap_pi(self, theta):
        """Return angle in [-π, π)."""
        return (theta + np.pi) % (2*np.pi) - np.pi

    def filter_samples(self):
        if self.polar_samples is None or self.polar_samples.size == 0:
            return

        r     = self.polar_samples[:, 0]
        theta = self.polar_samples[:, 1]

        finite_mask = np.isfinite(r)

        theta_wrapped = (theta + np.pi) % (2*np.pi) - np.pi           # [-π, π)
        front_mask    = np.abs(theta_wrapped) <= np.pi/2              # ±90°

        range_mask    = (r >= MIN_DIST_FILTER) & (r <= MAX_DIST_FILTER)

        keep = finite_mask & front_mask & range_mask
        self.polar_samples = self.polar_samples[keep]        

    def publish_samples(self):
        msg                   = LaserScan()
        msg.header.frame_id   = "laser"
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.angle_min         = ANGLE_MIN
        msg.angle_max         = ANGLE_MAX
        msg.angle_increment   = ANGLE_INCREMENT
        msg.range_min         = RANGE_MIN
        msg.range_max         = RANGE_MAX

        ranges = [float('inf')] * 1080             # native floats already

        for r, theta in self.polar_samples:
            index = int(round((theta - ANGLE_MIN) / ANGLE_INCREMENT))
            if 0 <= index < len(ranges):
                ranges[index] = float(r)           # ← cast NumPy scalar to Python float

        msg.ranges = ranges                        # passes the assertion now
        self.filtered_scan_pub.publish(msg)

    # def get_centroids(self, clusters):
    #     labels = np.unique(clusters.labels_)
    #     labels = labels[labels != -1]

    #     centroids = np.zeros((len(labels), self.carthesian_samples.shape[1]))

    #     for i, label in enumerate(labels):
    #         centroids[i] = np.mean(self.carthesian_samples[clusters.labels_ == label])

    #     return centroids

    def get_centroids(self, clusters):
        labels = np.unique(clusters.labels_)
        labels = labels[labels != -1]

        if len(labels) == 0:
            self.get_logger().warn("No valid clusters detected.")
            return np.empty((0, 2))

        # Tamaño de cada cluster
        cluster_sizes = [(label, np.sum(clusters.labels_ == label)) for label in labels]
        self.get_logger().info(f"Cluster sizes: {cluster_sizes}")

        # Seleccionar los dos más grandes
        top_labels = [label for label, _ in sorted(cluster_sizes, key=lambda x: -x[1])[:2]]
        
        centroids = []
        for label in top_labels:
            cluster_points = self.carthesian_samples[clusters.labels_ == label]
            centroids.append(np.mean(cluster_points, axis=0))

        return np.array(centroids)

    # def samples_to_polar(self, msg):
    #     angle_left = ANGLE_MAX*2*3/4
    #     ranges_left = msg.ranges[810:]
    #     polar_left = []

    #     for r in ranges_left:
    #         angle_left += ANGLE_INCREMENT
    #         if angle_left <= ANGLE_MAX*2:
    #             polar_left.append(np.array([r, angle_left]))
    #         else:
    #             polar_left.append(np.array([r, ANGLE_MAX*2 - 1e-18]))

    #     angle_right = 0
    #     ranges_right = msg.ranges[:270]
    #     polar_right = []
        
    #     for r in ranges_right:
    #         angle_right += ANGLE_INCREMENT
    #         if angle_right <= ANGLE_MAX/2:
    #             polar_right.append(np.array([r, angle_right]))
    #         else:
    #             polar_right.append(np.array([r, (ANGLE_MAX/2) - 1e-18]))

    #     self.polar_samples = np.array(polar_left + polar_right)

    def samples_to_polar(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)              # (1080,)
        idx    = np.arange(ranges.size, dtype=np.float32)

        angles = ANGLE_MIN + idx * ANGLE_INCREMENT                     # (1080,)

        self.polar_samples = np.column_stack((ranges, angles))         # (1080, 2)

    
    def polar_to_carthesian(self):
        r = self.polar_samples[:, 0]
        theta = self.polar_samples[:, 1]
        
        # Calculate x and y coordinates using vectorized operations
        x = -r * np.cos(theta)
        y = r * np.sin(theta)
        
        # Stack x and y to create the Cartesian coordinates
        self.carthesian_samples = np.column_stack((x, y))

    def carthesian_to_polar(self, carthesian_points):
            
        x = carthesian_points[:, 0]
        y = carthesian_points[:, 1]
        
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, -x)
    
        theta = np.mod(theta, 2*np.pi)
    
        polar_samples = np.column_stack((r, theta))
        
        return polar_samples

    def publish_centroids(self, cluster_centers):
        polar_centroids = self.carthesian_to_polar(cluster_centers)
        msg = self.centroid_to_laserscan(polar_centroids)
        self.centroids_pub.publish(msg)
        

    def centroid_to_laserscan(self, cluster_centers):
        msg = LaserScan()
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = ANGLE_INCREMENT
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg._header._frame_id= "laser"

        ranges = [0.0] * 1080

        # indexes = []

        for centroid in cluster_centers:
            rng = centroid[0]
            ang = centroid[1]
            index = int(round(ang / ANGLE_INCREMENT))
            # indexes.append(index)
            if index >= 1080:
                index = 1080-1

            ranges[index] = rng

        # print(f"indexes = {indexes}")
        msg.ranges = ranges

        return msg

def main(args=None):
    rclpy.init(args=args)
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()