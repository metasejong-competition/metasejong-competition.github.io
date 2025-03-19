import omni.graph.core as og
from pxr import Gf, UsdGeom
import numpy as np
from scipy.spatial.transform import Rotation as R
import omni.replicator.core as rep
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.prims import is_prim_path_valid, get_prim_at_path, set_prim_property
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.syntheticdata._syntheticdata as sd
import omni.usd
from omni.isaac.ros2_bridge import read_camera_info

def set_camera_absolute_orientation(camera_prim_path, euler_angles, degrees=True, order="zyx"):
    camera_prim = get_prim_at_path(camera_prim_path)
    if camera_prim is None:
        return
    quaternion = R.from_euler(order, euler_angles, degrees=degrees).as_quat()
    x, y, z, w = quaternion
    gf_quat = Gf.Quatd(w, Gf.Vec3d(x, y, z))
    set_prim_property(camera_prim_path, "xformOp:orient", gf_quat)

def set_camera_horizontal_aperture(camera_prim_path: str, aperture_mm: float):
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    if not camera_prim.IsValid():
        return
    usd_camera = UsdGeom.Camera(camera_prim)
    usd_camera.GetHorizontalApertureAttr().Set(aperture_mm)
    desired_aspect_ratio = 16 / 9  
    new_vertical_aperture = aperture_mm / desired_aspect_ratio
    usd_camera.GetVerticalApertureAttr().Set(new_vertical_aperture)

def get_camera_intrinsics(camera_prim_path):
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    if not camera_prim.IsValid():
        raise ValueError(f"Camera path {camera_prim_path} is invalid.")
    usd_camera = UsdGeom.Camera(camera_prim)  
    sensor_width = usd_camera.GetHorizontalApertureAttr().Get()
    sensor_height = usd_camera.GetVerticalApertureAttr().Get()
    return sensor_width, sensor_height

def publish_rgb(camera: Camera, freq, name):
    render_product = camera._render_product_path
    step_size = 1
    topic_name = "image_raw"
    queue_size = 100
    node_namespace = f"{name}"
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

def publish_camera_info(camera: Camera, freq, name):
    render_product = camera._render_product_path
    step_size = 1
    topic_name = "camera_info"
    queue_size = 100
    node_namespace = name
    frame_id = camera.prim_path.split("/")[-1]

    camera_prim_path = camera.prim_path
    sensor_width, sensor_height = get_camera_intrinsics(camera_prim_path)

    focal_length = 3.556
    camera_info = read_camera_info(render_product_path=render_product)

    fx = (focal_length / sensor_width) * camera_info["width"]
    fy = (focal_length / sensor_height) * camera_info["height"]
    cx, cy = camera_info["width"] / 2, camera_info["height"] / 2
    k_matrix = np.array([fx, 0, cx,  0, fy, cy,  0, 0, 1], dtype=np.float32)

    writer = rep.writers.get("ROS2PublishCameraInfo")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=k_matrix.reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

def publish_camera_tf(camera: Camera):
    camera_prim = camera.prim_path
    if not is_prim_path_valid(camera_prim):
        raise ValueError(f"Camera path '{camera_prim}' is invalid.")

    try:
        camera_frame_id = camera_prim.split("/")[-1]
        ros_camera_graph_path = "/CameraTFActionGraph"

        if not is_prim_path_valid(ros_camera_graph_path):
            og.Controller.edit(
                {
                    "graph_path": ros_camera_graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("IsaacClock", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("RosPublisher", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                        ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                    ]
                }
            )

        og.Controller.edit(
            ros_camera_graph_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishTF_"+camera_frame_id, "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("PublishRawTF_"+camera_frame_id+"_world", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTF_"+camera_frame_id+".inputs:topicName", "/tf"),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:topicName", "/tf"),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:parentFrameId", camera_frame_id),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:childFrameId", camera_frame_id+"_world"),
                    ("PublishRawTF_"+camera_frame_id+"_world.inputs:rotation", [0.5, -0.5, 0.5, 0.5]),
                ],
                og.Controller.Keys.CONNECT: [
                    (ros_camera_graph_path+"/OnTick.outputs:tick", "PublishTF_"+camera_frame_id+".inputs:execIn"),
                    (ros_camera_graph_path+"/OnTick.outputs:tick", "PublishRawTF_"+camera_frame_id+"_world.inputs:execIn"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime", "PublishTF_"+camera_frame_id+".inputs:timeStamp"),
                    (ros_camera_graph_path+"/IsaacClock.outputs:simulationTime", "PublishRawTF_"+camera_frame_id+"_world.inputs:timeStamp"),
                ],
            },
        )
    except Exception as e:
        print(e)

    set_target_prims(
        primPath=ros_camera_graph_path+"/PublishTF_"+camera_frame_id,
        inputName="inputs:targetPrims",
        targetPrimPaths=[camera_prim],
    )
