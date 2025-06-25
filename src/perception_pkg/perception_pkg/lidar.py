#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from chungungo_interfaces.msg import CloseBuoysCentroids
from std_msgs.msg import String
from sklearn.cluster import DBSCAN, OPTICS

import numpy as np

# ------ LiDAR Parameters ------
TIME_INCREMENT = 0.00011
ANGLE_INCREMENT = 0.008714509196579456  #? radians
ANGLE_MIN = -3.1241390705108643
ANGLE_MAX = 3.1415927410125732
RANGE_MIN = 0.15
RANGE_MAX = 12.0
N_SAMPLES = 720

# ------ Clustering parameters ------
EPS = 0.01  #? Distance (meters) between two points to be considered in the same cluster
CLUSTER_MIN_SAMPLES = 6
CLUSTER_MAX_SAMPLES = 30
MIN_DIST_FILTER = 0.3
MAX_DIST_FILTER = 0.8
    
class Lidar(Node): 
    def __init__(self):
        super().__init__("Lidar")

        # -------- Publishers and Subscribers --------
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.lidar_scan_cb, 1)
        self.filtered_scan_pub = self.create_publisher(LaserScan, "/filtered_scan", 1)
        self.centroids_pub = self.create_publisher(CloseBuoysCentroids, "/centroids", 1)
        self.centroids_ls_pub = self.create_publisher(LaserScan, "/centroids_laserscan", 1)
        
        # -------- Atributes --------
        self.polar_samples = None
        self.filtered_polar_samples = None
        self.cartesian_samples = None
        self.clustering = OPTICS(eps=EPS, min_samples=CLUSTER_MIN_SAMPLES)

    def lidar_scan_cb(self, msg: LaserScan):  #? 720 samples
        self.detect_buoys(msg)

    def detect_buoys(self, msg):    
        self.polar_samples = self.samples_to_polar(msg.ranges)
        self.filtered_polar_samples = self.filter_polar_samples(self.polar_samples)
        self.publish_samples_laserscan(self.filtered_polar_samples, topic="filtered_scan")
        self.cartesian_samples = self.polar_to_cartesian(self.filtered_polar_samples)
        # print(f"\nCarthesian Samples\n{self.cartesian_samples}")
        
        
        n_pts = self.cartesian_samples.shape[0]
        if n_pts >= CLUSTER_MIN_SAMPLES:  # if there is something to cluster   
            print("Clustering...")
            clusters = self.clustering.fit(self.cartesian_samples)
            centroids = self.get_centroids(clusters)
            # print(f"Centroids:\n{centroids}")
            polar_centroids = self.cartesian_to_polar(centroids)
            self.publish_centroids(polar_centroids)
            # print(f"Polar Centroids:\n{polar_centroids}")
            self.publish_samples_laserscan(polar_centroids, topic="centroids_laserscan")

        else:
            self.get_logger().debug(f"Only {n_pts} pts < min_samples={CLUSTER_MIN_SAMPLES} – skip clustering")

    def samples_to_polar(self, ranges):
        ranges = np.asarray(ranges, dtype=np.float32)
        thetas = np.arange(ranges.size, dtype=np.float32) * ANGLE_INCREMENT
        polar_samples = np.column_stack((ranges, thetas))
        
        return polar_samples
        
    def filter_polar_samples(self, raw_samples):
        #? Filter only front samples
        leftmost_angle = int(round(N_SAMPLES * 3/4))
        rightmost_angle = int(round(N_SAMPLES * 1/4))

        front_samples = np.vstack((raw_samples[leftmost_angle:], raw_samples[:rightmost_angle+1]))
        
        #? Filter samples by distance
        distance_mask = ((front_samples[:, 0] >= MIN_DIST_FILTER) & (front_samples[:, 0] <= MAX_DIST_FILTER))
        
        filtered_samples = front_samples[distance_mask]

        return filtered_samples
    
    def polar_to_cartesian(self, polar_samples):
        r = polar_samples[:, 0]
        theta = polar_samples[:, 1]
        
        # Calculate x and y coordinates using vectorized operations
        x = r * np.sin(theta)   # +X is down
        y = r * np.cos(theta)   # +Y is right
        
        # Stack x and y to create the Cartesian coordinates
        cartesian_samples = np.column_stack((x, y))

        return cartesian_samples

    def cartesian_to_polar(self, cartesian_points):  
        x = cartesian_points[:, 0]
        y = cartesian_points[:, 1]

        r     = np.hypot(x, y)          # √(x² + y²)
        theta = np.arctan2(x, y)        # note the swapped order!
        theta = np.mod(theta, 2*np.pi)  # map to [0, 2π)

        return np.column_stack((r, theta))
    
    def get_centroids(self, clusters, k: int = 2) -> np.ndarray:
        labels = clusters.labels_
        valid_mask = labels >= 0

        if not valid_mask.any():
            self.get_logger().warn("No valid clusters detected.")
            return np.empty((0, 2), dtype=float)

        # ---- cluster sizes ---------------------------------------------------
        sizes = np.bincount(labels[valid_mask])
        keep = np.where((0 < sizes) & (sizes <= CLUSTER_MAX_SAMPLES))[0]

        if keep.size == 0:
            self.get_logger().info(f"All clusters exceed samples cap = {CLUSTER_MAX_SAMPLES}")
            return np.empty((0, 2), dtype=float)


        centroids_all = np.vstack([self.cartesian_samples[labels == l].mean(axis=0) for l in keep])
        # ranked = keep[np.argsort(sizes[keep])[::-1]]           # labels in descending size
        
        # distance of each centroid from the sensor
        dists = np.linalg.norm(centroids_all, axis=1)

        # order by smallest distance (nearest objects first)
        order = np.argsort(dists)
        top_idx   = order[:k]            # indices within 'keep' / 'centroids_all'
        top_labels = keep[top_idx]
        centroids  = centroids_all[top_idx]
        

        # ----- logging -------------------------------------------------------
        cluster_info = [(int(lbl), int(sizes[lbl]), float(dists[i])) for i, lbl in enumerate(top_labels)]
        # self.get_logger().info(f"Clusters kept (label, size, range): {cluster_info}")

        return centroids

    def publish_samples_laserscan(self, samples, topic: str):
        msg                   = LaserScan()
        msg.header.frame_id   = "laser"
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.angle_min         = ANGLE_MIN
        msg.angle_max         = ANGLE_MAX
        msg.angle_increment   = ANGLE_INCREMENT
        msg.range_min         = RANGE_MIN
        msg.range_max         = RANGE_MAX

        ranges = [12.0] * N_SAMPLES

        for r, theta in samples:
            index = int(round(theta/ANGLE_INCREMENT))
            if 0 <= index < N_SAMPLES:
                ranges[index] = float(r)

        msg.ranges = ranges

        if topic == "filtered_scan":
            self.filtered_scan_pub.publish(msg)
        elif topic == "centroids_laserscan":
            self.centroids_ls_pub.publish(msg)
        else:
            self.get_logger().error("WRONG TOPIC")

    def publish_centroids(self, centroids):
        if centroids.size == 0:
            self.get_logger().debug("No centroids - nothing to publish.")
            return                                    #  <-- salir sin publicar

        msg = CloseBuoysCentroids()

        # 1) primer centroide (existe siempre que size > 0)
        msg.centroid_1.range = float(centroids[0, 0])
        msg.centroid_1.theta = float(centroids[0, 1])

        # 2) segundo centroide, sólo si hay ≥ 2
        if centroids.shape[0] > 1:
            msg.centroid_2.range = float(centroids[1, 0])
            msg.centroid_2.theta = float(centroids[1, 1])
 
        self.centroids_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
