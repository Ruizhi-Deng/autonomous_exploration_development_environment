import rclpy
from rclpy.node import Node
import time

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
from .map_predictor import MapPredictor
import numpy as np
from sklearn.cluster import DBSCAN
import std_msgs
from sensor_msgs_py import point_cloud2
import torch
import cv2
from prob_msgs.msg import ProbGrid

class GridPredictor(Node):
    def __init__(self):
        """
        initialize the node
        - subscribe to /visible_map
        - set up a timer
        - publish /predicted_map (including confidence)
        - publish all frontiers (with cluster)
        """
        super().__init__("grid_predictor_node")
        # subscribe to visible map
        self.visible_map_subscription = self.create_subscription(
            OccupancyGrid, "/visible_map", self.visibleMapCallback, 10
        )

        self.declare_parameter("prediction_cycle", 2.0)
        prediction_cycle = self.get_parameter("prediction_cycle").get_parameter_value().double_value

        self.declare_parameter("enable_uncertainty", False)
        enable_uncertainty = self.get_parameter("enable_uncertainty").get_parameter_value().bool_value

        self.declare_parameter("correct_prediction", True)
        self.correct_prediction = self.get_parameter("correct_prediction").get_parameter_value().bool_value
        
        self.declare_parameter("wrong_prediction_confidence", 0.3)
        self.wrong_prediction_confidence = self.get_parameter("wrong_prediction_confidence").get_parameter_value().double_value
        
        self.timer = self.create_timer(prediction_cycle, self.timerCallback)

        # publish predicted map
        self.predicted_map_publisher = self.create_publisher(
            OccupancyGrid, "/predicted_map", 10
        )

        self.predicted_confidence_publisher = self.create_publisher(
            ProbGrid, "/confidence_map", 10
        )

        # initialize visible map and predicted map
        self.visible_map = None
        self.predicted_map = None
        self.confidence_map = None

        # initialize frontiers list
        self.frontiers = []
        self.clustered_frontiers = []

        # publish frontiers
        self.frontier_publisher = self.create_publisher(
            PointCloud2, "/all_frontier", 10
        )

        # load predictor model
        self.declare_parameter("model_path", "")
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        package_share = get_package_share_directory("grid_predictor")
        model_path = package_share + "/" + model_path
        self.predictor = MapPredictor(model_path, enable_uncertainty)

        self.declare_parameter("enable_prediction", True)
        self.enable_prediction = self.get_parameter("enable_prediction").get_parameter_value().bool_value

        self.declare_parameter("frontier_cluster_eps", 0.2)
        self.frontier_cluster_eps = self.get_parameter("frontier_cluster_eps").get_parameter_value().double_value

        self.declare_parameter("frontier_cluster_min_samples", 1)
        self.frontier_cluster_min_samples = self.get_parameter("frontier_cluster_min_samples").get_parameter_value().integer_value

        self.get_logger().info("Is CUDA Used? " + str(torch.cuda.is_available()))
        self.get_logger().info("Current device: " + str(next(self.predictor.net.parameters()).device))

    def timerCallback(self):
        if self.visible_map is None:
            self.get_logger().info("No visible map received yet")
            return

        self.get_logger().info("Starting Grid Prediction Stage")
        t_start = time.time()

        if not self.enable_prediction:
            self.predicted_map = self.numpyToOccupancyGrid(np.full((self.visible_map.info.height, self.visible_map.info.width), 100), self.visible_map)
            self.confidence_map = self.numpyToConfidenceGrid(np.zeros((self.visible_map.info.height, self.visible_map.info.width)), self.visible_map)
            self.predicted_map_publisher.publish(self.predicted_map)
            self.predicted_confidence_publisher.publish(self.confidence_map)
            return

        # predict map
        global_map_np = self.occupancyGrid2Numpy(self.visible_map)
        frontier_np = self.worldToGrid(self.clustered_frontiers, self.visible_map)
        predicted, predicted_confidence = self.predictor.predictMap(global_map_np, frontier_np)
        
        # Apply wrong prediction if correct_prediction is False
        if not self.correct_prediction:
            # Selectively flip 50% of free cells and 50% of occupied cells
            flipped = predicted.copy()
            
            # Find free cells (255) and occupied cells (0)
            free_mask = (predicted == 255)
            occupied_mask = (predicted == 0)
            
            # Get indices of free and occupied cells
            free_indices = np.where(free_mask)
            occupied_indices = np.where(occupied_mask)
            
            # Randomly select 16.6% of free cells to flip to occupied
            if len(free_indices[0]) > 0:
                num_free_to_flip = len(free_indices[0]) // 6
                flip_free_idx = np.random.choice(len(free_indices[0]), num_free_to_flip, replace=False)
                flipped[free_indices[0][flip_free_idx], free_indices[1][flip_free_idx]] = 0
            
            # Randomly select 16.6% of occupied cells to flip to free
            if len(occupied_indices[0]) > 0:
                num_occupied_to_flip = len(occupied_indices[0])
                flip_occupied_idx = np.random.choice(len(occupied_indices[0]), num_occupied_to_flip, replace=False)
                flipped[occupied_indices[0][flip_occupied_idx], occupied_indices[1][flip_occupied_idx]] = 255
            
            predicted = flipped
            
            # Set confidence to a low value for all predicted cells
            predicted_confidence = np.full_like(predicted_confidence, self.wrong_prediction_confidence)
            self.get_logger().info(f"Using mixed wrong prediction: 16.6% free->occupied, 16.6% occupied->free, confidence {self.wrong_prediction_confidence}")
        
        # Copy visible map values to predicted map (overwrite predictions with known values)
        # This ensures observed areas are not affected by wrong predictions
        visible_known_mask = (global_map_np != 100)  # Not unknown (100 is unknown)
        predicted[visible_known_mask] = global_map_np[visible_known_mask]
        predicted_confidence[visible_known_mask] = 1.0  # High confidence for observed areas
        
        self.predicted_map = self.numpyToOccupancyGrid(predicted, self.visible_map)
        self.confidence_map = self.numpyToConfidenceGrid(predicted_confidence, self.visible_map)
        self.predicted_map_publisher.publish(self.predicted_map)
        self.predicted_confidence_publisher.publish(self.confidence_map)

        t_end = time.time()
        duration_ms = (t_end - t_start) * 1000
        self.get_logger().info(f"Grid Prediction Stage completed in {duration_ms:.2f} ms")

    def isFrontier(self, x, y, grid):
        """
        check if cell (x, y) is a frontier:
        - must be free cell
        - must have at least one unknown neighbour
        """
        if grid[y, x] != 0:
            return False
    
        height, width = grid.shape
        neighbors = [
            (y - 1, x),
            (y + 1, x),
            (y, x - 1),
            (y, x + 1),
            (y - 1, x - 1),
            (y - 1, x + 1),
            (y + 1, x - 1),
            (y + 1, x + 1),
        ]
        count = 0
        for ny, nx in neighbors:
            if 0 <= ny < height and 0 <= nx < width:
                if grid[ny, nx] == -1:
                    count = count + 1
                if count >= 2:
                    return True
        return False
    
    def clusterFrontiers(self, frontiers, eps, min_samples):
        """
        Cluster frontiers with DBSCAN, pick closest-to-centroid frontier as representative.
        """
        if not frontiers:
            return []

        points = np.array(frontiers)[:, :2]
        clustered = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustered.labels_
        rep_points = []

        for lbl in set(labels):
            if lbl == -1:  # noise cluster, skip
                continue
            cluster_pts = np.array(frontiers)[labels == lbl]
            centroid = np.mean(cluster_pts[:, :2], axis=0)

            # find cluster point closest to centroid
            dists = np.linalg.norm(cluster_pts[:, :2] - centroid, axis=1)
            closest = cluster_pts[np.argmin(dists)]
            rep_points.append(tuple(closest))

        return rep_points
    
    def publishFrontiers(self, frontiers, publisher):
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        publisher.publish(point_cloud2.create_cloud_xyz32(header, frontiers))
    
    def visibleMapCallback(self, msg: OccupancyGrid):
        if not msg.data or msg.info.width == 0 or msg.info.height == 0:
            self.get_logger().warn("Received empty OccupancyGrid, skipping.")
            return
        
        self.visible_map = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        free_mask = (grid == 0).astype(np.uint8)
        unknown_mask = (grid == -1).astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, kernel)
        frontier_mask = (free_mask == 1) & (dilated_unknown == 1)
        y_coords, x_coords = np.where(frontier_mask)
        
        self.frontiers = []
        for x, y in zip(x_coords, y_coords):
            wx = origin.position.x + (x + 0.5) * resolution
            wy = origin.position.y + (y + 0.5) * resolution
            wz = origin.position.z
            self.frontiers.append((wx, wy, wz))
        
        if len(self.frontiers) > 0:
            self.clustered_frontiers = self.clusterFrontiers(self.frontiers, eps=self.frontier_cluster_eps, min_samples=self.frontier_cluster_min_samples)
            self.publishFrontiers(self.clustered_frontiers, self.frontier_publisher)
        else:
            self.clustered_frontiers = []

    def occupancyGrid2Numpy(self, occupancy_grid: OccupancyGrid):
        """
        Convert OccupancyGrid.data to numpy.ndarray with dtype=np.uint8 and size=(H, W).
        Mapping:
        - Free cells (0) -> 255
        - Occupied cells (100) -> 0
        - Unknown cells (-1) -> 100
        """
        if not occupancy_grid.data:
            return None
        
        # Convert to numpy array and reshape to (H, W)
        grid_array = np.array(occupancy_grid.data, dtype=np.int8)
        height = occupancy_grid.info.height
        width = occupancy_grid.info.width
        grid_2d = grid_array.reshape((height, width))
        
        # Create output array with uint8 dtype
        result = np.zeros((height, width), dtype=np.uint8)
        
        # Apply mapping
        result[grid_2d == 0] = 255    # Free cells -> 255
        result[grid_2d == 100] = 0    # Occupied cells -> 0
        result[grid_2d == -1] = 100   # Unknown cells -> 100
        
        return result

    def worldToGrid(self, world_points, occupancy_grid):
        """
        Convert a list of points from world coordinates to grid index coordinates.
        
        Args:
            world_points: List of tuples (x, y) or (x, y, z) in world coordinates
            occupancy_grid: OccupancyGrid message containing map info (resolution, origin)
            
        Returns:
            List of tuples (grid_x, grid_y) representing grid indices
        """
        if not occupancy_grid or not world_points:
            return []
        
        resolution = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin.position
        
        grid_indices = []
        for point in world_points:
            # Extract x, y coordinates (ignore z if present)
            wx, wy = point[0], point[1]
            
            # Convert world coordinates to grid indices
            grid_x = int((wx - origin.x) / resolution)
            grid_y = int((wy - origin.y) / resolution)
            
            grid_indices.append((grid_x, grid_y))
        
        return grid_indices

    def numpyToOccupancyGrid(self, grid_array, reference_grid):
        """
        Convert numpy.ndarray to OccupancyGrid message.
        Mapping:
        - 255 -> Free cells (0)
        - 100 -> Unknown cells (-1)
        - 0 -> Occupied cells (100)
        
        Args:
            grid_array: numpy.ndarray with values 0, 100, 255
            reference_grid: OccupancyGrid message to copy info from
            
        Returns:
            OccupancyGrid message with current time as stamp and "map" as frame_id
        """
        if grid_array is None or reference_grid is None:
            return None
        
        # Create output OccupancyGrid
        occupancy_grid = OccupancyGrid()
        
        # Copy info from reference grid
        occupancy_grid.info = reference_grid.info
        
        # Set header with current time and map frame
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        
        # Create empty data array
        height, width = grid_array.shape
        data = np.zeros(height * width, dtype=np.int8)
        
        # Apply mapping
        data[grid_array.flatten() == 255] = 0    # Free cells -> 0
        data[grid_array.flatten() == 100] = -1   # Unknown cells -> -1
        data[grid_array.flatten() == 0] = 100    # Occupied cells -> 100
        
        # Convert to list and assign
        occupancy_grid.data = data.tolist()
        
        return occupancy_grid
    
    def numpyToConfidenceGrid(self, grid_array, reference_grid):
        """
        Convert numpy.ndarray to ProbGrid message
        Set confidence to 1.0 for cells that are free or occupied in visible_map
        """
        if grid_array is None or reference_grid is None:
            return None

        # Create output ProbGrid
        prob_grid = ProbGrid()

        # Copy info
        prob_grid.info = reference_grid.info

        # Set header
        prob_grid.header.stamp = self.get_clock().now().to_msg()
        prob_grid.header.frame_id = "map"

        # Get the visible map data to identify known cells
        visible_map_data = np.array(reference_grid.data, dtype=np.int8)
        height, width = grid_array.shape

        # Create confidence data from predicted values
        confidence_data = grid_array.flatten()

        # Set confidence to 1.0 for cells that are free (0) or occupied (100) in visible_map
        # Unknown cells (-1) keep their predicted confidence
        known_cells_mask = (visible_map_data >= 0)  # Free (0) or occupied (100) cells
        confidence_data[known_cells_mask] = 1.0

        # Apply mapping
        prob_grid.data = confidence_data.tolist()

        return prob_grid

def main(args=None):
    # initialize node
    rclpy.init(args=args)
    node = GridPredictor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()