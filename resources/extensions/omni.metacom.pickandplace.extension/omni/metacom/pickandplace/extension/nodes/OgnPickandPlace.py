"""
This is the implementation of the OGN node defined in OgnPickandPlace.ogn
"""

import numpy as np
import carb
import os
from pathlib import Path
import traceback

from scipy.spatial.transform import Rotation as R
import omni.graph.core as og
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.core.articulations import Articulation
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from typing import Optional
from omni.isaac.debug_draw import _debug_draw

from omni.isaac.core.scenes import Scene
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import quat_to_rot_matrix

SCALE_FACTOR = 0.05


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(
        self,
        robot_articulation: Articulation,
        robot_description_path: str,
        urdf_path: str,
        end_effector_frame_name: Optional[str] = None,
    ) -> None:
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=robot_description_path,
            urdf_path=urdf_path,
        )
        end_effector_frame_name = "end_effector_link"
        ArticulationKinematicsSolver.__init__(
            self, robot_articulation, self._kinematics, end_effector_frame_name
        )

def convert_quat_xyzw_to_wxyz(q: np.ndarray) -> np.ndarray:
    # q: [x, y, z, w] -> [w, x, y, z]
    return np.array([q[3], q[0], q[1], q[2]])

def get_translate_from_prim(
    translation_from_source: np.ndarray,
    source_prim,
    target_info: tuple[np.ndarray, np.ndarray],
) -> np.ndarray:
    robot_prim = XFormPrim("/World/metacombot/scout_v2_base/base_link")
    robot_pos , robot_orient = robot_prim.get_world_pose()
    rotation_matrix = R.from_quat(robot_orient).as_matrix()
    translation_rotated = rotation_matrix @ np.array([0.2649, 0.0, 0.11293])

    cur_target_object_pos, current_target_object_ori = target_info

    T_robot2world = np.eye(4, dtype=float)
    T_robot2world[0:3, 0:3] = quat_to_rot_matrix(robot_orient)
    T_robot2world[0:3, 3] = robot_pos + translation_rotated + np.array([0.0, 0.0, 0.22194])
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_lines()
    draw.draw_points(
        [robot_pos + translation_rotated + np.array([0.0, 0.0, 0.22194])],
        [(1.0, 0.0, 0.0, 1.0)],
        [100000],
    )
    T_obj2world = np.eye(4, dtype=float)
    T_obj2world[0:3, 0:3] = quat_to_rot_matrix(current_target_object_ori) * SCALE_FACTOR
    T_obj2world[0:3, 3] = cur_target_object_pos

    T_robot2obj = np.linalg.inv(T_robot2world) @ T_obj2world

    target_pos = np.pad(translation_from_source, (0, 1), constant_values=1)

    target_cube_relative_coordinates = (target_pos @ np.transpose(T_robot2obj))[0:3]

    return target_cube_relative_coordinates

# def get_translate_from_prim(
#     translation_from_source: np.ndarray,
#     source_prim,
#     target_info: tuple[np.ndarray, np.ndarray],
# ) -> np.ndarray:
#     """
#     특정 프림(source_prim)을 기준으로 한 translation_from_source가
#     실제 월드 좌표계에서 어떻게 변환되는지 계산하여 반환합니다.
#     """
#     robot_pos, robot_orient = source_prim.get_world_pose()
#     cur_target_object_pos, current_target_object_ori = target_info

#     T_robot2world = np.eye(4, dtype=float)
#     T_robot2world[0:3, 0:3] = quat_to_rot_matrix(robot_orient)
#     T_robot2world[0:3, 3] = robot_pos

#     T_obj2world = np.eye(4, dtype=float)
#     T_obj2world[0:3, 0:3] = quat_to_rot_matrix(current_target_object_ori) * SCALE_FACTOR
#     T_obj2world[0:3, 3] = cur_target_object_pos

#     T_robot2obj = np.linalg.inv(T_robot2world) @ T_obj2world

#     target_pos = np.pad(translation_from_source, (0, 1), constant_values=1)

#     target_cube_relative_coordinates = (target_pos @ np.transpose(T_robot2obj))[0:3]

#     return target_cube_relative_coordinates


class OgnPickNPlaceInit(BaseResetNode):
    def __init__(self):
        self.initialized = None
        self.robot_prim_path = None
        self.ee_prim_path = None
        self.robot_name = None
        self.ee_name = None
        self.manipulator = None
        self.robot_controller = None
        self.robot_description_path = None
        self.urdf_path = None

        self.waypoints = None
        self.current_waypoint_idx = 0
        self.previous_actions = None
        self.task = "Idle"
        self.timestamp = 0

        super().__init__(initialize=False)

    def initialize_scene(self):
        self.scene = Scene()
        self.manipulator = SingleManipulator(
            prim_path=self.robot_prim_path,
            name=self.robot_name,
            end_effector_prim_name=self.ee_name,
        )
        self.scene.add(self.manipulator)
        self.manipulator.initialize()
        self.robot = self.scene.get_object(self.robot_name)
        self.robot_controller = KinematicsSolver(
            robot_articulation=self.robot,
            robot_description_path=self.robot_description_path,
            urdf_path=self.urdf_path,
        )
        self.articulation_controller = self.robot.get_articulation_controller()
        self.initialized = True
        self.timestamp = 0
        self.previous_actions = None
        self.waypoints = None
        return

    def custom_reset(self):
        self.robot_controller = None

def move_along_waypoints(
    state: OgnPickNPlaceInit,
    robot_xform_prim: XFormPrim,
    start_offset: np.ndarray,
    end_offset: np.ndarray,
    reference_info: tuple[np.ndarray, np.ndarray],
    orientation: np.ndarray,
    num_steps: int,
    next_task_name: str,
) -> bool:
    """
    - state.waypoints가 None이면 초기화해서 start_offset에서 end_offset 방향으로 num_steps만큼 웨이포인트를 생성
    - 현재 waypoint로 IK를 구해 로봇을 이동
    - 모든 waypoint를 소화하면 다음 state.task를 next_task_name으로 바꾸고 True를 반환 (상태 전환 신호)
    - IK에 실패하면 False 반환
    """
    # 웨이포인트가 없다면 초기화
    if state.waypoints is None:
        start_point = np.round(
            get_translate_from_prim(start_offset, robot_xform_prim, reference_info),
            decimals=3,
        )
        end_point = np.round(
            get_translate_from_prim(end_offset, robot_xform_prim, reference_info),
            decimals=3,
        )
        state.waypoints = [
            start_point + (end_point - start_point) * i / (num_steps - 1)
            for i in range(num_steps)
        ]
        state.current_waypoint_idx = 0
        print(f"  -> Waypoints initialized: {state.waypoints}")

    # 현재 이동할 웨이포인트 추출
    cur_target = state.waypoints[state.current_waypoint_idx]
    # IK 계산
    actions, succ = state.robot_controller.compute_inverse_kinematics(
        target_position=np.array(cur_target), target_orientation=orientation
    )
    print(f"  -> IK success={succ}")

    # 성공 시 로봇 이동
    if succ:
        # ex) 매 3번 스텝마다만 apply_action 호출
        state.timestamp += 1
        if state.timestamp % 3 == 0:
            state.articulation_controller.apply_action(actions)
            state.current_waypoint_idx += 1

        print(f"  -> Reached waypoint! Next idx={state.current_waypoint_idx}")
        if state.current_waypoint_idx >= len(state.waypoints):
            # 모든 웨이포인트를 완료했으면 다음 태스크로 전환
            state.task = next_task_name
            state.waypoints = None
            state.current_waypoint_idx = 0
            return True
    else:
        carb.log_warn(f"IK did not converge to a solution for target {cur_target}")
        return False

    return False


class OgnPickandPlace:
    @staticmethod
    def internal_state():
        return OgnPickNPlaceInit()

    @staticmethod
    def compute(db) -> bool:
        TOLERANCE = 1e-1
        state = db.per_instance_state
        ee_down_orientation = np.array(
            [0, 1, 0, 0]
        )  # 엔드 이펙터가 아래로 향하도록 설정
        APART_TRANSLATE = 7
        CENTER_TRANSLATE = 2.7
        num_steps = 20

        try:
            if not state.initialized:
                # 로봇 초기화
                if db.inputs.robot_prim_path == "":
                    carb.log_warn("robot prim path is not set")
                    return False
                SELECTED_ROBOT = "kinova_robot"
                state.robot_prim_path = db.inputs.robot_prim_path
                state.ee_name = "end_effector_link"

                state.robot_name = SELECTED_ROBOT
                robot_cfg_path = Path(__file__).parent.parent / "robot" / SELECTED_ROBOT
                state.robot_description_path = os.path.join(
                    robot_cfg_path, SELECTED_ROBOT + "_descriptor.yaml"
                )
                state.urdf_path = os.path.join(robot_cfg_path, SELECTED_ROBOT + ".urdf")
                state.task = "Idle"  # 초기 상태
                state.initialize_scene()
                carb.log_info("Robot Initialized")

            current_target_object_ori = db.inputs.grasping_point_ori
            cur_target_object_pos = db.inputs.grasping_point_pos

            # quaternion 순서를 변환: XYZW -> WXYZ
            current_target_object_ori = convert_quat_xyzw_to_wxyz(
                np.array(current_target_object_ori)
            )

            current_target_obj_info = (cur_target_object_pos, current_target_object_ori)

            object_target_ori = db.inputs.placement_point_ori
            object_target_pos = db.inputs.placement_point_pos

            object_target_ori = convert_quat_xyzw_to_wxyz(np.array(object_target_ori))

            placement_of_target_obj_info = (object_target_pos, object_target_ori)

            # 물체와 로봇 위치 가져오기
            robot_xform_prim = XFormPrim(state.robot_prim_path)

            # ========== 로그: 상태 표시 ==========
            print(f"\n[RobotPickNPlace] Current State: {state.task}")

            # 상태머신 로직
            if state.task == "Idle":
                # Idle 상태에서 바로 물체 위로 이동 상태로 전환
                print("  -> Transition to MoveAboveCube")
                db.outputs.gripper_grasp_command = False
                state.task = "MoveAboveCube"

            elif state.task == "MoveAboveCube":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True

                translation_from_source = np.array([0, 0, -APART_TRANSLATE])
                target_cube_relative_coordinates = get_translate_from_prim(
                    translation_from_source,
                    robot_xform_prim,
                    current_target_obj_info
                )

                actions, succ = state.robot_controller.compute_inverse_kinematics(
                    target_position=target_cube_relative_coordinates,
                    target_orientation=current_target_object_ori,
                )
                print(f"  -> IK Result: succ={succ}")

                if succ:
                    state.previous_actions = actions
                    state.articulation_controller.apply_action(actions)
                    print("  -> Applying IK action, moving to ReachDown")
                else:
                    carb.log_warn("Failed to move above cube.")
                    print(traceback.format_exc())

                joint_pos_dict = state.robot.get_joint_positions()
                if isinstance(joint_pos_dict, dict):
                    current_joint_positions = np.array(joint_pos_dict.joint_positions).flatten()
                elif isinstance(joint_pos_dict, (list, np.ndarray)):
                    current_joint_positions = np.array(joint_pos_dict).flatten()
                else:
                    raise TypeError(f"Unexpected type: {type(joint_pos_dict)}")

                robot_dof_names = state.robot.dof_names
                target_dof_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
                # 원본 순서를 유지하면서 index 찾기
                indices = [robot_dof_names.index(name) for name in target_dof_names if name in robot_dof_names]
                joint_pos_dict = joint_pos_dict[indices]
                # indices를 이용해 정확한 joint_positions를 뽑아냄
                prev_actions = state.previous_actions.joint_positions

                # 길이 확인하여 최소 길이로 맞추기
                min_length = min(len(current_joint_positions), len(prev_actions))
                current_joint_positions = current_joint_positions[indices]
                prev_actions = prev_actions[:min_length]

                if np.allclose(current_joint_positions, prev_actions, atol=TOLERANCE):
                    print(f"  -> Joint positions match previous action (tolerance {TOLERANCE}), transitioning to ReachDown")
                    state.task = "ReachDown"

            elif state.task == "ReachDown":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True
                # APART_TRANSLATE -> CENTER_TRANSLATE 구간을 여러 웨이포인트로 이동
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -APART_TRANSLATE]),
                    end_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    reference_info=current_target_obj_info,
                    orientation=current_target_object_ori,
                    num_steps=num_steps,
                    next_task_name="Pick",
                )
                # 완료되면 Pick 상태로 전환

            elif state.task == "Pick":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True
                print("  -> In Pick state: closing gripper")
                db.outputs.gripper_grasp_command = True
                print("  -> Transition to PickUpObject")
                state.task = "PickUpObject"

            elif state.task == "PickUpObject":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True
                # CENTER_TRANSLATE -> APART_TRANSLATE (역방향)으로 여러 웨이포인트 이동
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    end_offset=np.array([0, 0, -APART_TRANSLATE]),
                    reference_info=current_target_obj_info,
                    orientation=current_target_object_ori,
                    num_steps=num_steps,
                    next_task_name="MoveToTargetAbove",
                )
                # 완료되면 MoveToTargetAbove 상태로 전환

            elif state.task == "MoveToTargetAbove":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True
                # 오브젝트를 옮길 위치 위로 이동
                translation_from_source = np.array([0, 0, -APART_TRANSLATE])
                target_cube_relative_coordinates = get_translate_from_prim(
                    translation_from_source,
                    robot_xform_prim,
                    placement_of_target_obj_info,
                )
                actions, succ = state.robot_controller.compute_inverse_kinematics(
                    target_position=target_cube_relative_coordinates,
                    target_orientation=ee_down_orientation,
                )
                print(f"  -> IK Result: succ={succ}")
                if succ:
                    state.previous_actions = actions
                    state.articulation_controller.apply_action(actions)
                    print("  -> Applying IK action, moving to ReachDown")
                else:
                    carb.log_warn("Failed to move above cube.")
                    print(traceback.format_exc())

                joint_pos_dict = state.robot.get_joint_positions()
                if isinstance(joint_pos_dict, dict):
                    current_joint_positions = np.array(
                        joint_pos_dict.joint_positions
                    ).flatten()
                elif isinstance(joint_pos_dict, (list, np.ndarray)):
                    current_joint_positions = np.array(joint_pos_dict).flatten()
                else:
                    raise TypeError(f"Unexpected type: {type(joint_pos_dict)}")

                robot_dof_names = state.robot.dof_names
                target_dof_names = [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5",
                    "joint_6",
                ]
                # 원본 순서를 유지하면서 index 찾기
                indices = [
                    robot_dof_names.index(name)
                    for name in target_dof_names
                    if name in robot_dof_names
                ]
                joint_pos_dict = joint_pos_dict[indices]
                # indices를 이용해 정확한 joint_positions를 뽑아냄
                prev_actions = state.previous_actions.joint_positions

                # 길이 확인하여 최소 길이로 맞추기
                min_length = min(len(current_joint_positions), len(prev_actions))
                current_joint_positions = current_joint_positions[indices]
                prev_actions = prev_actions[:min_length]

                if np.allclose(current_joint_positions, prev_actions, atol=TOLERANCE):
                    print(
                        f"  -> Joint positions match previous action (tolerance {TOLERANCE}), transitioning to ReachDown"
                    )
                    state.task = "ReachDownTarget"

            elif state.task == "ReachDownTarget":
                db.outputs.pick_and_place_command = True
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                # APART_TRANSLATE -> CENTER_TRANSLATE 구간을 여러 웨이포인트로 이동 (목적지)
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -APART_TRANSLATE]),
                    end_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    reference_info=placement_of_target_obj_info,
                    orientation=object_target_ori,
                    num_steps=num_steps,
                    next_task_name="Place",
                )
                # 완료되면 Place 상태로 전환

            elif state.task == "Place":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True
                print("  -> In Place state: opening gripper")
                db.outputs.gripper_grasp_command = False
                print("  -> Transition to MoveAwayFromTarget")
                state.task = "MoveAwayFromTarget"

            elif state.task == "MoveAwayFromTarget":
                db.outputs.execute = og.ExecutionAttributeState.ENABLED
                db.outputs.pick_and_place_command = True
                # CENTER_TRANSLATE -> APART_TRANSLATE (역방향) 이동
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    end_offset=np.array([0, 0, -APART_TRANSLATE]),
                    reference_info=placement_of_target_obj_info,
                    orientation=object_target_ori,
                    num_steps=num_steps,
                    next_task_name="Done",
                )
                # 완료되면 Done 상태로 전환

            elif state.task == "Done":
                # 모든 동작이 완료된 상태면 picknplacereq를 false로 기록하여 다음 실행 때 다시 대기하도록 함
                db.outputs.pick_and_place_command = False
                state.task = "Idle"

        except Exception as error:
            db.log_warn(str(error))
            print(traceback.format_exc())
            return False

        return True
