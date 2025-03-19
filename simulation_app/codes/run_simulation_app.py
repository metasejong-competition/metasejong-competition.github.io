# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
import sys

import numpy as np
from isaacsim import SimulationApp

# Example ROS2 bridge sample demonstrating the manual loading of stages
# and creation of ROS components
ISSAC_INIT_CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(ISSAC_INIT_CONFIG)

import carb
import omni.graph.core
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions, stage
from omni.isaac.nucleus import get_assets_root_path
##

import numpy as np
import yaml
from omni.isaac.sensor import Camera

from pxr import UsdPhysics, UsdGeom, Sdf
import omni.kit.commands
import omni.usd
import carb



######################## APP INIT #########################
if os.environ.get("MODE") == "local" or os.environ.get("MODE") is None:
    home_dir = os.path.expanduser("~")
    ROBOT_STAGE_PATH = "/world/robot"
    ROBOT_USD_PATH = os.path.join(home_dir, "Documents/metasejong-competition.github.io/resources/models/robot/assembled_robots/metacom.usd")
    BACKGROUND_STAGE_PATH = "/world/background"
    BACKGROUND_USD_PATH = os.path.join(home_dir, "Documents/metasejong-competition.github.io/resources/models/meta-sejong/2024_SejongUniv.usd")
    CONFIG_YAML_PATH = os.path.join(home_dir, "Documents/metasejong-competition.github.io/resources/env_config/config_local.yaml")
    OBJECT_GROUND_TRUTH_DIR = os.path.join(home_dir, "Documents/metasejong-competition.github.io/resources/output")
elif os.environ.get("MODE") == "dev":
    ROBOT_STAGE_PATH = "/world/robot"
    ROBOT_USD_PATH = "/root/Documents/robot/assembled_robots/metacom.usd"
    BACKGROUND_STAGE_PATH = "/world/background"
    BACKGROUND_USD_PATH = '/root/Documents/meta-sejong/2024_SejongUniv.usd'
    CONFIG_YAML_PATH = "/root/Documents/env_config/config.yaml"
    OBJECT_GROUND_TRUTH_DIR = "/root/Documents/output"
elif os.environ.get("MODE") == "prod":
    ROBOT_STAGE_PATH = "/world/robot"
    ROBOT_USD_PATH = "/root/Documents/robot/assembled_robots/metacom.usd"
    BACKGROUND_STAGE_PATH = "/world/background"
    BACKGROUND_USD_PATH = '/root/Documents/meta-sejong/2024_SejongUniv.usd'
    CONFIG_YAML_PATH = "/root/Documents/env_config/config.yaml"
else:
    raise ValueError("MODE is invalid")

# YAML 파일 읽기
with open(CONFIG_YAML_PATH, "r") as file:
    env_config_data = yaml.safe_load(file)

# 변수로 불러오기
camera_configs = env_config_data['camera_configs']
standard_pos = env_config_data['standard_pos']
object_list = env_config_data['object_list']
asset_attribute = env_config_data['asset_attribute']
robot_start_point = np.array(env_config_data["robot_start_point"])
view_point_camera_view_eye = env_config_data["view_point_camera_view_eye"]
view_point_camera_view_target = env_config_data["view_point_camera_view_target"]

object_camera_resolution = env_config_data["object_camera_resolution"]

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")
extensions.enable_extension("aisl.omnigraph.extension")

simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)
# assets_root_path = get_assets_root_path()
# if assets_root_path is None:
#     carb.log_error("Could not find Isaac Sim assets folder")
#     simulation_app.close()
#     sys.exit()

from world_utils import setup_world
# 함수 호출
setup_world(view_point_camera_view_eye, view_point_camera_view_target, BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
simulation_app.update()
######################## APP INIT #########################




######################## WORLD INIT #########################
usd_stage = omni.usd.get_context().get_stage()
stage_prims = usd_stage.GetPrimAtPath(BACKGROUND_STAGE_PATH)
collision_api = UsdPhysics.CollisionAPI.Apply(stage_prims)
collision_api.GetCollisionEnabledAttr().Set(True)

xform = UsdGeom.Xformable(stage_prims)
xform.AddTranslateOp().Set((0.0, 0.0, -13.0))
xform.AddScaleOp().Set((0.01, 0.01, 0.01))

dome_light = usd_stage.DefinePrim("/world/DomeLight", "DomeLight")
dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)
######################## WORLD INIT #########################




######################## OBJECT INIT ########################
from object_utils import set_friction, publish_objects, set_object_attributes_semantics, set_object_physics

set_friction()
simulation_app.update()

objects_prim_paths = publish_objects(asset_attribute, OBJECT_GROUND_TRUTH_DIR, stage, robot_start_point, standard_pos, object_list)
simulation_app.update()

set_object_attributes_semantics(asset_attribute, standard_pos, object_list)
simulation_app.update()

set_object_physics(objects_prim_paths)
simulation_app.update()
######################## OBJECT INIT ########################




######################## CAMERA INIT ########################
from camera_utils import publish_rgb, publish_camera_info, publish_camera_tf, set_camera_absolute_orientation, set_camera_horizontal_aperture

cam_dict = []
for cam in camera_configs:
    camera = Camera(
        prim_path="/world/"+cam['name'],
        position=np.array(cam['position']),
        frequency=1,
        resolution=(object_camera_resolution[0], object_camera_resolution[1]),
    )
    set_camera_absolute_orientation("/world/"+cam['name'], cam['rotation'], degrees=True, order="zyx")
    set_camera_horizontal_aperture("/world/"+cam['name'], 30)
    cam_dict.append(camera)

for cam in cam_dict:
    cam.initialize()
simulation_app.update()

approx_freq = 120
for idx, cam in enumerate(cam_dict):
    publish_rgb(cam, approx_freq, camera_configs[idx]['name'])
    publish_camera_info(cam, approx_freq, camera_configs[idx]['name'])
    publish_camera_tf(cam)
######################## CAMERA INIT ########################





######################## ROBOT INIT #########################
from robot_utils import publish_robot, set_robot_arm_material

publish_robot(ROBOT_USD_PATH, robot_start_point)
set_robot_arm_material()
######################## ROBOT INIT #########################




simulation_app.update()
simulation_context.initialize_physics()
simulation_context.play()
while simulation_app.is_running():
    simulation_context.step(render=True)
simulation_context.stop()
simulation_app.close()