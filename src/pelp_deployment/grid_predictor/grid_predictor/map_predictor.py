import torch
import numpy as np
import copy
import matplotlib.pyplot as plt
from .fpunet import FPUNet
import torch.nn.functional as F
import cv2

"""
In cv2, we have
- 0 for occupied
- 100 for unknown
- 255 for free
"""

class MapPredictor:
    def __init__(self, model_path, IF_UNCERTAIN) -> None:
        self.device = "cuda"
        self.origin_map_size = 128  # 128 * 128
        self.net = FPUNet(in_channels=3, n_classes=3, feature_scale=2).to(self.device)
        state_dict = torch.load(model_path, weights_only=True)
        self.net.load_state_dict(state_dict)
        self.last_predicted_map = None
        self.floor_plan = None
        self.IF_UNCERTAIN = IF_UNCERTAIN

        # check whether gpu is using
        print(f"Is CUDA available: {torch.cuda.is_available()}")
        print(f"Current device: {next(self.net.parameters()).device}")
        
    def predictMap(self, input_map, now_points, debug=False):
        now_map = copy.deepcopy(input_map)
        h, w = now_map.shape
        already_known_mask = (now_map != 100)
        
        known_confidence = np.zeros((h, w), dtype=np.float32)
        known_confidence[already_known_mask] = 1.0
        
        self.last_predicted_map = copy.deepcopy(now_map)
        self.last_predicted_confidence = copy.deepcopy(known_confidence)

        if len(now_points) == 0:
            return self.last_predicted_map, self.last_predicted_confidence

        # 16 or 32 is good, depend on GPU memory
        batch_size = 16

        # make points into several groups
        for i in range(0, len(now_points), batch_size):
            sub_points = now_points[i : i + batch_size]

            # 1. Local slicing
            local_maps_list = [self.getLocalMap(now_map, pt, self.origin_map_size) for pt in sub_points]
            batch_tensor = torch.from_numpy(np.array(local_maps_list)).unsqueeze(1)

            # 2. Preprocessing and transfer
            one_hot_batch = self.label2OneHot(batch_tensor).float().to(self.device)

            # 3. Inference
            with torch.no_grad():
                predicted_outputs = self.net(one_hot_batch)
                predicted_outputs = predicted_outputs.cpu()

                # print(f"\n--- Batch {i // batch_size} Debug Info ---")
                # print(f"Raw Logits (predicted_outputs):")
                # print(f"  Shape: {predicted_outputs.shape}")
                # print(f"  Min: {predicted_outputs.min().item():.4f}")
                # print(f"  Max: {predicted_outputs.max().item():.4f}")
                # print(f"  Mean: {predicted_outputs.mean().item():.4f}")

                if predicted_outputs.max() > 10:
                    print("  [WARNING] Logits are very high (>10), Softmax will be ~1.0 everywhere.")
            
            # 4. Manually clear cache
            torch.cuda.empty_cache()

            # 5. Result fusion
            batch_labels = self.probability2Label(predicted_outputs).numpy()
            batch_confidences = self.probability2Confidence(predicted_outputs).numpy()

            for j, now_point in enumerate(sub_points):
                pred_label = batch_labels[j]
                pred_conf = batch_confidences[j]

                re_map = np.full((h, w), 100, dtype=np.uint8)
                predicted_map_of = self.toOriginalFrame(pred_label, now_point, (h, w), re_map)
                
                re_conf = np.zeros((h, w), dtype=np.float32)
                predicted_conf_of = self.toOriginalFrame(pred_conf, now_point, (h, w), re_conf)

                predicted_map_of[already_known_mask] = now_map[already_known_mask]
                predicted_conf_of[already_known_mask] = known_confidence[already_known_mask]

                update_mask = (predicted_map_of != 100)
                self.last_predicted_map[update_mask] = predicted_map_of[update_mask]
                self.last_predicted_confidence[update_mask] = predicted_conf_of[update_mask]

        return self.last_predicted_map, self.last_predicted_confidence
    
    def probability2Label(self, origin):
        res = torch.ones(
            origin.shape[0], origin.shape[2], origin.shape[3], dtype=torch.int64
        )
        for i in range(origin.shape[0]):
            index_mat = torch.argmax(origin[i], dim=0)
            res[i] = index_mat
            res[i][index_mat == 0] = 255
            res[i][index_mat == 1] = 0
            res[i][index_mat == 2] = 100
        return res
    
    def probability2Confidence(self, origin):
        if self.IF_UNCERTAIN:
            scale_factor = 2.0 
            scaled_logits = origin * scale_factor
            probs = F.softmax(scaled_logits, dim=1)
            conf, _ = torch.max(probs, dim=1)
        else:
            conf, _ = torch.max(origin, dim=1)
        return conf
    
    def label2OneHot(self, data):
        # three cases below represent free, occupied, unknown in order
        res = torch.zeros((data.shape[0], 3, data.shape[2], data.shape[3]))
        for i in range(data.shape[0]):
            res[i, 0] = data[i] > 200
            res[i, 1] = data[i] < 10
            # > 90 and < 110
            res[i, 2] = (data[i] > 90) & (data[i] < 110)
        return res
    
    def toOriginalFrame(self, new_map, ori_point, origin_map_size, re_map):
        """
        Copy local prediction back to the original map frame.

        new_map: Local prediction with shape (height, width) = (y_size, x_size)
        ori_point: (grid_x, grid_y) center point in global map
        origin_map_size: (height, width) of the global map
        re_map: The global map to copy results into
        """
        map_size = new_map.shape
        local_map_size = int(map_size[0] // 2)

        # Unpack ori_point
        goal_x, goal_y = ori_point[0], ori_point[1]

        # Global map dimensions
        map_height, map_width = origin_map_size[0], origin_map_size[1]

        # Calculate slice boundaries in global map
        # re_map[row, col] = re_map[y, x]
        start_y = max(0, goal_y - local_map_size)
        end_y = min(map_height, goal_y + local_map_size)
        start_x = max(0, goal_x - local_map_size)
        end_x = min(map_width, goal_x + local_map_size)

        # Calculate offsets in local map
        offset_y_start = max(0, local_map_size - goal_y)
        offset_y_end = min(2 * local_map_size, map_height - goal_y + local_map_size)
        offset_x_start = max(0, local_map_size - goal_x)
        offset_x_end = min(2 * local_map_size, map_width - goal_x + local_map_size)

        # Copy from local to global: re_map[y, x] = new_map[y, x]
        re_map[start_y:end_y, start_x:end_x] = new_map[
            offset_y_start:offset_y_end, offset_x_start:offset_x_end
        ]

        return re_map
    
    def expandObstacle(self, image, obs_index=0, unknown_index=100, free_index=255, kernel_size=5):
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        obs_space = (image == obs_index).astype(np.uint8)
        free_bool = image == free_index

        expanded_obs = cv2.dilate(obs_space, kernel).astype(bool)
        obs = expanded_obs & (~free_bool)
        new_image = copy.deepcopy(image)
        new_image[obs] = obs_index
        return new_image
    
    def img2FloorPlan(self, img):
        edges = cv2.Canny(img, 50, 200, apertureSize=3)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=10, minLineLength=5, maxLineGap=3
        )
        index_map = np.zeros(img.shape, dtype=np.uint8)

        for i in range(len(lines)):
            cv2.line(
                index_map,
                (lines[i, 0, 0], lines[i, 0, 1]),
                (lines[i, 0, 2], lines[i, 0, 3]),
                1,
                2,
            )

        floor_plan_img = copy.deepcopy(img)

        not_plan_index = np.logical_and(img == 0, index_map == 0)
        floor_plan_img[not_plan_index] = 255

        return floor_plan_img
    
    def getLocalMap(self, global_map, next_goal, local_map_size, defailt_value=255):
        """
        Extract a local map centered at next_goal from the global map.

        global_map shape: (height, width) = (y_size, x_size)
        next_goal: (grid_x, grid_y) where grid_x is column index, grid_y is row index
        """
        # Unpack next_goal
        goal_x, goal_y = next_goal[0], next_goal[1]

        # Map dimensions (height=y, width=x)
        map_height, map_width = global_map.shape[0], global_map.shape[1]

        # Calculate slice boundaries in global map
        # global_map[row, col] = global_map[y, x]
        start_y = max(0, goal_y - local_map_size)
        end_y = min(map_height, goal_y + local_map_size)
        start_x = max(0, goal_x - local_map_size)
        end_x = min(map_width, goal_x + local_map_size)

        # Create local map filled with default value
        local_map = np.full(
            (2 * local_map_size, 2 * local_map_size), defailt_value, dtype=np.uint8
        )

        # Calculate offsets in local map
        offset_y_start = max(0, local_map_size - goal_y)
        offset_y_end = min(2 * local_map_size, map_height - goal_y + local_map_size)
        offset_x_start = max(0, local_map_size - goal_x)
        offset_x_end = min(2 * local_map_size, map_width - goal_x + local_map_size)

        # Copy from global to local: local_map[y, x] = global_map[y, x]
        local_map[offset_y_start:offset_y_end, offset_x_start:offset_x_end] = global_map[
            start_y:end_y, start_x:end_x
        ]

        return local_map
    
    def thresholdSegmentation(self, img, threshold_list, target_value):
        # segment the img into n value
        # for i-th value: it will be larger than threshold_list[i] and <= threshold_list[i+1]
        new_img = copy.deepcopy(img)
        for i in range(len(threshold_list) - 1):
            index = np.logical_and(img >= threshold_list[i], img <= threshold_list[i + 1])
            new_img[index] = target_value[i]

        return new_img