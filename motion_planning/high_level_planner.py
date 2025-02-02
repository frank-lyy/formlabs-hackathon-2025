from pydrake.all import (
    RigidTransform,
    RotationMatrix,
)
from manipulation.meshcat_utils import AddMeshcatTriad

from string_segmentor import get_feature_idx, get_position_index, get_position_from_index

import yaml
import numpy as np

class HighLevelPlanner():
    def __init__(self, plant, plant_context, left_eef_frame, right_eef_frame):
        self.plant = plant
        self.plant_context = plant_context
        self.left_eef_frame = left_eef_frame
        self.right_eef_frame = right_eef_frame
        
        # Load YAML file
        with open("high_level_plan.yaml", "r") as file:
            data = yaml.safe_load(file)
            
        self.steps = []
        self.max_step_num = 0
        for step in data["steps"]:
            step_data = {
                "step_id": step["step_id"],
                "description": step["description"],
                "left_reference_point_cloud": step.get("left_reference_point_cloud", None),
                "right_reference_point_cloud": step.get("right_reference_point_cloud", None),
                "l_target_pose": step["l_target_pose"],
                "r_target_pose": step["r_target_pose"],
                "l_open_close": step["l_open_close"],
                "r_open_close": step["r_open_close"]
            }

            # Store processed step
            self.steps.append(step_data)
            
            self.max_step_num = step["step_id"]
    
        self.step_num = 0
            
    
    def transform_point_camera_to_world_frame(self, x):
        """
        x is an array of 3 coordinates. Translates x from Zed's left camera optical
        frame to world frame.
        """
        X_cam = RigidTransform(RotationMatrix(), x)
        world_T_cam = self.plant.CalcRelativeTransform(self.plant_context, self.plant.world_frame(), self.plant.GetFrameByName("zed2i_left_camera_optical_frame"))
        X_world = world_T_cam @ X_cam
        return X_world.translation()

    def get_next_action(self):
        if self.step_num > self.max_step_num:  # all actions performed
            return None, None, None, None
        
        step_data = self.steps[self.step_num]
        left_reference_point_cloud = np.array(step_data["left_reference_point_cloud"])
        right_reference_point_cloud = np.array(step_data["right_reference_point_cloud"])
        
        # Execution environment (isolated namespace)
        exec_namespace = {
            "np": np,
            "RigidTransform": RigidTransform,
            "RotationMatrix": RotationMatrix,
            "plant": self.plant,
            "plant_context": self.plant_context,
            "p": self,
            "get_feature_idx": get_feature_idx,
            "get_position_index": get_position_index,
            "get_position_from_index": get_position_from_index,
            "transform_point_camera_to_world_frame": self.transform_point_camera_to_world_frame,
            "left_eef_frame": self.left_eef_frame,
            "right_eef_frame": self.right_eef_frame,
            "left_reference_point_cloud": left_reference_point_cloud,
            "right_reference_point_cloud": right_reference_point_cloud,
        }
        
        exec(step_data["l_target_pose"], exec_namespace)
        l_target_pose = exec_namespace.get("l_target_pose", None)

        exec(step_data["r_target_pose"], exec_namespace)
        r_target_pose = exec_namespace.get("r_target_pose", None)
        
        l_open_close = step_data["l_open_close"]
        r_open_close = step_data["r_open_close"]
            
        self.step_num += 1
        
        return l_target_pose, r_target_pose, l_open_close, r_open_close
