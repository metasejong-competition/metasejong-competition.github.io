import omni.graph.core as og
from omni.isaac.core.utils import prims
##
import numpy as np
from omni.kit.commands import execute

def publish_robot(usd_path, robot_start_point):
    # 메타컴봇 생성
    prims.create_prim(
        "/world/metacombot",
        "Xform",
        position=robot_start_point,
        usd_path=usd_path,
    )

def set_robot_arm_material():
    execute(
        "BindMaterial",
        material_path="/World/ObjectFriction",
        prim_path=["/World/metacombot/metacom_robot/robotiq_hand_e/Robotiq_Hand_E_edit/Robotiq_Hand_E/left_gripper/D_A03_ASM_DOIGTS_PARALLELES_1ROBOTIQ_HAND_E_DEFEATURE_02", "/World/metacombot/metacom_robot/robotiq_hand_e/Robotiq_Hand_E_edit/Robotiq_Hand_E/right_gripper/D_A03_ASM_DOIGTS_PARALLELES_1ROBOTIQ_HAND_E_DEFEATURE_02"],
        strength=["weakerThanDescendants", "weakerThanDescendants"],
        material_purpose="",
    )
    
    try:
        og.Controller.edit(
            {"graph_path": "/pick_and_place", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    ("gripper_open_value", "omni.graph.nodes.ConstantDouble"),
                    ("gripper_close_value", "omni.graph.nodes.ConstantDouble"),
                    ("gripper_select_if", "omni.graph.nodes.SelectIf"),
                    ("gripper_write_prim_attribute", "omni.graph.nodes.WritePrimAttribute"),
                    ("gripper_attribute_name", "omni.graph.nodes.ConstantToken"),
                    ("gripper_path", "omni.graph.nodes.ConstantToken"),
                    ("pickandplace_recv_custom_event", "omni.graph.action.OnCustomEvent"),
                    ("manipulator_path", "omni.graph.nodes.ConstantToken"),
                    ("pick_and_place_node", "aisl.omnigraph.extension.PickandPlaceNode"),
                    ("pickandplace_read_attribute", "omni.graph.core.ReadVariable"),
                    ("pickandplace_write_attribute", "omni.graph.core.WriteVariable"),
                    ("boolean_or", "omni.graph.nodes.BooleanOr"),
                    ("pickandplace_command_not", "omni.graph.nodes.ConstantToken"),
                    ("pickandplace_command", "omni.graph.nodes.ConstantToken"),
                    ("pickandplace_select_if", "omni.graph.nodes.SelectIf"),
                    ("pickandplace_send_custom_event", "omni.graph.action.SendCustomEvent"),
                    ("orientation_downward", "omni.graph.nodes.ConstantDouble4"),
                    (
                        "orientation_revert_x_as_pi",
                        "aisl.omnigraph.extension.reverted_orientation",
                    ),
                    ("end_point_pos", "omni.graph.nodes.ConstantDouble3"),
                    ("picking_point_bias", "omni.graph.nodes.ConstantDouble3"),
                    ("break_picking_point_bias", "omni.graph.nodes.BreakVector3"),
                    ("add_x_bias", "omni.graph.nodes.Add"),
                    ("add_y_bias", "omni.graph.nodes.Add"),
                    ("add_z_bias", "omni.graph.nodes.Add"),
                    ("make_picking_point", "omni.graph.nodes.MakeVector3"),
                    ("break_target_point", "omni.graph.nodes.BreakVector3"),
                    ("read_target_pos", "omni.graph.nodes.ReadPrimAttribute"),
                    ("read_target_ori", "omni.graph.nodes.ReadPrimAttribute"),
                    ("target_pos_attribute_name", "omni.graph.nodes.ConstantToken"),
                    ("target_ori_attribute_name", "omni.graph.nodes.ConstantToken"),
                    (
                        "command_receiver",
                        "aisl.omnigraph.extension.PickandPlaceCommandReceiver",
                    ),
                    ("bot_label_visualizer", "omni.graph.visualization.nodes.DrawLabel"),
                    (
                        "target_obj_label_visualizer",
                        "omni.graph.visualization.nodes.DrawLabel",
                    ),
                    (
                        "plastic_bin_label_visualizer",
                        "omni.graph.visualization.nodes.DrawLabel",
                    ),
                    (
                        "paper_bin_label_visualizer",
                        "omni.graph.visualization.nodes.DrawLabel",
                    ),
                    (
                        "aluminum_bin_label_visualizer",
                        "omni.graph.visualization.nodes.DrawLabel",
                    ),
                    (
                        "bot_world_transform",
                        "omni.graph.nodes.GetPrimLocalToWorldTransform",
                    ),
                    (
                        "target_obj_world_transform",
                        "omni.graph.nodes.GetPrimLocalToWorldTransform",
                    ),
                    (
                        "plastic_bin_world_transform",
                        "omni.graph.nodes.GetPrimLocalToWorldTransform",
                    ),
                    (
                        "paper_bin_world_transform",
                        "omni.graph.nodes.GetPrimLocalToWorldTransform",
                    ),
                    (
                        "aluminum_bin_world_transform",
                        "omni.graph.nodes.GetPrimLocalToWorldTransform",
                    ),
                    (
                        "bot_prim_path",
                        "omni.graph.nodes.ConstantToken",
                    ),
                    (
                        "plastic_bin_prim_path",
                        "omni.graph.nodes.ConstantToken",
                    ),
                    (
                        "paper_bin_prim_path",
                        "omni.graph.nodes.ConstantToken",
                    ),
                    (
                        "aluminum_bin_prim_path",
                        "omni.graph.nodes.ConstantToken",
                    ),
                    (
                        "font_size",
                        "omni.graph.nodes.ConstantFloat",
                    ),
                    (
                        "big_font_size",
                        "omni.graph.nodes.ConstantFloat",
                    ),
                    (
                        "translate_vec", "aisl.omnigraph.extension.translate_placement_node"
                    ),
                ],
                og.Controller.Keys.CREATE_VARIABLES: [
                    ("pickandplace", "bool"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "gripper_attribute_name.inputs:value",
                        "drive:linear:physics:targetVelocity",
                    ),
                    (
                        "gripper_path.inputs:value",
                        "/World/metacombot/metacom_robot/robotiq_hand_e/Robotiq_Hand_E_edit/Robotiq_Hand_E/Slider_1",
                    ),
                    ("gripper_write_prim_attribute.inputs:usePath", True),
                    ("gripper_open_value.inputs:value", 0.035),
                    ("gripper_close_value.inputs:value", -0.1),
                    ("pickandplace_read_attribute.inputs:graph", "/pick_and_place"),
                    ("pickandplace_read_attribute.inputs:variableName", "pickandplace"),
                    ("pickandplace_write_attribute.inputs:graph", "/pick_and_place"),
                    ("pickandplace_write_attribute.inputs:variableName", "pickandplace"),
                    ("pickandplace_recv_custom_event.inputs:eventName", "pickandplace"),
                    ("pickandplace_command.inputs:value", "pickandplace"),
                    ("pickandplace_command_not.inputs:value", "pickandplace_not"),
                    (
                        "manipulator_path.inputs:value",
                        "/World/metacombot/metacom_robot/kinova_robot_manipulator",
                    ),
                    ("orientation_downward.inputs:value", np.array([0.0, 0.0, 0.0, 1.0])),
                    ("end_point_pos.inputs:value", np.array([0.3, 0.0, 0.0])),
                    ("picking_point_bias.inputs:value", np.array([-0.00, 0.00, -0.00])),
                    ("target_pos_attribute_name.inputs:value", "xformOp:translate"),
                    ("target_ori_attribute_name.inputs:value", "xformOp:orient"),
                    ("read_target_pos.inputs:usePath", True),
                    ("read_target_ori.inputs:usePath", True),
                    ("bot_label_visualizer.inputs:text", "Metacombot"),
                    ("target_obj_label_visualizer.inputs:text", "Target Object"),
                    ("plastic_bin_label_visualizer.inputs:text", "Plastic Bin"),
                    ("paper_bin_label_visualizer.inputs:text", "Paper Bin"),
                    ("aluminum_bin_label_visualizer.inputs:text", "Aluminum Bin"),
                    (
                        "bot_prim_path.inputs:value",
                        "/World/metacombot/scout_v2_base/base_link",
                    ),
                    (
                        "plastic_bin_prim_path.inputs:value",
                        "/World/metacombot/recyclebins/green_plastic",
                    ),
                    (
                        "paper_bin_prim_path.inputs:value",
                        "/World/metacombot/recyclebins/blue_paper",
                    ),
                    (
                        "aluminum_bin_prim_path.inputs:value",
                        "/World/metacombot/recyclebins/red_aluminum",
                    ),
                    ("bot_label_visualizer.inputs:color", np.array([1.0, 0.0, 0.0, 1.0])),
                    (
                        "target_obj_label_visualizer.inputs:color",
                        np.array([0.0, 1.0, 1.0, 1.0]),
                    ),
                    (
                        "plastic_bin_label_visualizer.inputs:color",
                        np.array([0.0, 1.0, 0.0, 1.0]),
                    ),
                    (
                        "paper_bin_label_visualizer.inputs:color",
                        np.array([0.0, 0.0, 1.0, 1.0]),
                    ),
                    (
                        "aluminum_bin_label_visualizer.inputs:color",
                        np.array([1.0, 0.0, 0.0, 1.0]),
                    ),
                    ("bot_label_visualizer.inputs:offset", np.array([0.0, 0.0, 0.4])),
                    (
                        "target_obj_label_visualizer.inputs:offset",
                        np.array([0.0, 0.0, 0.3]),
                    ),
                    (
                        "plastic_bin_label_visualizer.inputs:offset",
                        np.array([0.0, 0.0, 0.2]),
                    ),
                    ("paper_bin_label_visualizer.inputs:offset", np.array([0.0, 0.0, 0.2])),
                    (
                        "aluminum_bin_label_visualizer.inputs:offset",
                        np.array([0.0, 0.0, 0.2]),
                    ),
                    ("bot_world_transform.inputs:usePath", True),
                    ("target_obj_world_transform.inputs:usePath", True),
                    ("plastic_bin_world_transform.inputs:usePath", True),
                    ("paper_bin_world_transform.inputs:usePath", True),
                    ("aluminum_bin_world_transform.inputs:usePath", True),
                    ("font_size.inputs:value", 20.0),
                    ("big_font_size.inputs:value", 40.0),
                ],
                og.Controller.Keys.CONNECT: [
                    ("on_playback_tick.outputs:tick", "bot_label_visualizer.inputs:execIn"),
                    (
                        "on_playback_tick.outputs:tick",
                        "target_obj_label_visualizer.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "plastic_bin_label_visualizer.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "paper_bin_label_visualizer.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "aluminum_bin_label_visualizer.inputs:execIn",
                    ),
                    (
                        "bot_world_transform.outputs:localToWorldTransform",
                        "bot_label_visualizer.inputs:transform",
                    ),
                    (
                        "target_obj_world_transform.outputs:localToWorldTransform",
                        "target_obj_label_visualizer.inputs:transform",
                    ),
                    (
                        "plastic_bin_world_transform.outputs:localToWorldTransform",
                        "plastic_bin_label_visualizer.inputs:transform",
                    ),
                    (
                        "paper_bin_world_transform.outputs:localToWorldTransform",
                        "paper_bin_label_visualizer.inputs:transform",
                    ),
                    (
                        "aluminum_bin_world_transform.outputs:localToWorldTransform",
                        "aluminum_bin_label_visualizer.inputs:transform",
                    ),
                    (
                        "bot_world_transform.outputs:localToWorldTransform",
                        "translate_vec.inputs:transformation",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "translate_vec.inputs:execIn",
                    ),
                    (
                        "bot_prim_path.inputs:value",
                        "bot_world_transform.inputs:primPath",
                    ),
                    (
                        "plastic_bin_prim_path.inputs:value",
                        "plastic_bin_world_transform.inputs:primPath",
                    ),
                    (
                        "paper_bin_prim_path.inputs:value",
                        "paper_bin_world_transform.inputs:primPath",
                    ),
                    (
                        "aluminum_bin_prim_path.inputs:value",
                        "aluminum_bin_world_transform.inputs:primPath",
                    ),
                    ("big_font_size.inputs:value", "bot_label_visualizer.inputs:size"),
                    (
                        "big_font_size.inputs:value",
                        "target_obj_label_visualizer.inputs:size",
                    ),
                    ("font_size.inputs:value", "plastic_bin_label_visualizer.inputs:size"),
                    ("font_size.inputs:value", "paper_bin_label_visualizer.inputs:size"),
                    ("font_size.inputs:value", "aluminum_bin_label_visualizer.inputs:size"),
                    (
                        "on_playback_tick.outputs:tick",
                        "gripper_write_prim_attribute.inputs:execIn",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "command_receiver.inputs:execIn",
                    ),
                    (
                        "command_receiver.outputs:prim_name",
                        "target_obj_world_transform.inputs:primPath",
                    ),
                    (
                        "gripper_open_value.inputs:value",
                        "gripper_select_if.inputs:ifTrue",
                    ),
                    (
                        "gripper_close_value.inputs:value",
                        "gripper_select_if.inputs:ifFalse",
                    ),
                    (
                        "gripper_select_if.outputs:result",
                        "gripper_write_prim_attribute.inputs:value",
                    ),
                    (
                        "gripper_attribute_name.inputs:value",
                        "gripper_write_prim_attribute.inputs:name",
                    ),
                    (
                        "gripper_path.inputs:value",
                        "gripper_write_prim_attribute.inputs:primPath",
                    ),
                    (
                        "pickandplace_recv_custom_event.outputs:execOut",
                        "pick_and_place_node.inputs:execution",
                    ),
                    (
                        "manipulator_path.inputs:value",
                        "pick_and_place_node.inputs:robot_prim_path",
                    ),
                    (
                        "pick_and_place_node.outputs:gripper_grasp_command",
                        "gripper_select_if.inputs:condition",
                    ),
                    (
                        "pick_and_place_node.outputs:pick_and_place_command",
                        "boolean_or.inputs:a",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "pickandplace_write_attribute.inputs:execIn",
                    ),
                    (
                        "command_receiver.outputs:pickandplace_command",
                        "pickandplace_write_attribute.inputs:value",
                    ),
                    ("pickandplace_read_attribute.outputs:value", "boolean_or.inputs:b"),
                    (
                        "boolean_or.outputs:result",
                        "pickandplace_select_if.inputs:condition",
                    ),
                    (
                        "pickandplace_command.inputs:value",
                        "pickandplace_select_if.inputs:ifTrue",
                    ),
                    (
                        "pickandplace_command_not.inputs:value",
                        "pickandplace_select_if.inputs:ifFalse",
                    ),
                    (
                        "pickandplace_select_if.outputs:result",
                        "pickandplace_send_custom_event.inputs:eventName",
                    ),
                    (
                        "on_playback_tick.outputs:tick",
                        "pickandplace_send_custom_event.inputs:execIn",
                    ),
                    (
                        "orientation_downward.inputs:value",
                        "orientation_revert_x_as_pi.inputs:orientation",
                    ),
                    (
                        "orientation_revert_x_as_pi.outputs:reverted_orientation",
                        "pick_and_place_node.inputs:grasping_point_ori",
                    ),
                    (
                        "orientation_revert_x_as_pi.outputs:reverted_orientation",
                        "pick_and_place_node.inputs:placement_point_ori",
                    ),
                    (
                        "end_point_pos.inputs:value",
                        "translate_vec.inputs:translate_vec",
                    ),
                    (
                        "translate_vec.outputs:result",
                        "pick_and_place_node.inputs:placement_point_pos",
                    ),
                    (
                        "make_picking_point.outputs:tuple",
                        "pick_and_place_node.inputs:grasping_point_pos",
                    ),
                    ("add_x_bias.outputs:sum", "make_picking_point.inputs:x"),
                    ("add_y_bias.outputs:sum", "make_picking_point.inputs:y"),
                    ("add_z_bias.outputs:sum", "make_picking_point.inputs:z"),
                    (
                        "picking_point_bias.inputs:value",
                        "break_picking_point_bias.inputs:tuple",
                    ),
                    ("break_picking_point_bias.outputs:x", "add_x_bias.inputs:a"),
                    ("break_picking_point_bias.outputs:y", "add_y_bias.inputs:a"),
                    ("break_picking_point_bias.outputs:z", "add_z_bias.inputs:a"),
                    ("break_target_point.outputs:x", "add_x_bias.inputs:b"),
                    ("break_target_point.outputs:y", "add_y_bias.inputs:b"),
                    ("break_target_point.outputs:z", "add_z_bias.inputs:b"),
                    (
                        "read_target_pos.outputs:value",
                        "break_target_point.inputs:tuple",
                    ),
                    (
                        "target_pos_attribute_name.inputs:value",
                        "read_target_pos.inputs:name",
                    ),
                    (
                        "command_receiver.outputs:prim_name",
                        "read_target_ori.inputs:primPath",
                    ),
                    (
                        "read_target_pos.outputs:value",
                        "break_target_point.inputs:tuple",
                    ),
                    (
                        "target_ori_attribute_name.inputs:value",
                        "read_target_ori.inputs:name",
                    ),
                    (
                        "command_receiver.outputs:prim_name",
                        "read_target_pos.inputs:primPath",
                    ),
                ],
            },
        )
    except Exception as e:
        print(e)

    # try:
    #     og.Controller.edit(
    #         {"graph_path": "/viz", "evaluator_name": "execution"},
    #         {
    #             og.Controller.Keys.CREATE_NODES: [
    #                 ('on_playback_tick', 'omni.graph.action.OnPlaybackTick'),
    #                 ('screen_space_text', 'omni.graph.visualization.nodes.DrawScreenSpaceText'),
    #                 ('read_recycling_score', 'omni.graph.nodes.ReadPrimAttribute'),
    #                 ('read_trash_pose_estimation_score', 'omni.graph.nodes.ReadPrimAttribute'),
    #                 ('attribute_name', 'omni.graph.nodes.ConstantToken'),
    #                 ('recycling_score_str', 'omni.graph.nodes.ConstantString'),
    #                 ('pose_estimation_score_str', 'omni.graph.nodes.ConstantString'),
    #                 ('tostr1', 'omni.graph.nodes.ToString'),
    #                 ('tostr2', 'omni.graph.nodes.ToString'),
    #                 ('appendstr1', 'omni.graph.nodes.AppendString'),
    #                 ('appendstr2', 'omni.graph.nodes.AppendString'),
    #                 ('appendstr3', 'omni.graph.nodes.AppendString'),
    #                 ('constant_double2', 'omni.graph.nodes.ConstantDouble2'),
    #                 ('constant_color4d', 'omni.graph.nodes.ConstantColor4d'),
    #                 ('constant_float', 'omni.graph.nodes.ConstantFloat'),
    #                 ('to_string', 'omni.graph.nodes.ToString'),
    #                 ('append_string', 'omni.graph.nodes.BuildString'),
    #                 ('constant_string', 'omni.graph.nodes.ConstantString'),
    #                 ('append_string_01', 'omni.graph.nodes.BuildString'),
    #                 ('to_int', 'omni.graph.nodes.ToInt'),
    #                 ('trashesReadAttrib', 'omni.graph.nodes.ReadPrimAttribute'),
    #                 ('to_string_01', 'omni.graph.nodes.ToString'),
    #                 ('append_string_02', 'omni.graph.nodes.BuildString'),
    #                 ('constant_string_01', 'omni.graph.nodes.ConstantString'),
    #                 ('append_string_03', 'omni.graph.nodes.BuildString'),
    #                 ('constant_string_02', 'omni.graph.nodes.ConstantString'),
    #                 ('append_string_04', 'omni.graph.nodes.BuildString'),
    #                 ('add', 'omni.graph.nodes.Add'),
    #                 ('to_string_02', 'omni.graph.nodes.ToString'),
    #                 ('constant_string_03', 'omni.graph.nodes.ConstantString'),
    #                 ('append_string_05', 'omni.graph.nodes.BuildString'),
    #                 ('append_string_06', 'omni.graph.nodes.BuildString'),
    #             ],

    #             og.Controller.Keys.CONNECT: [
    #                 ('constant_color4d.inputs:value', 'screen_space_text.inputs:backgroundColor'),
    #                 ('on_playback_tick.outputs:tick', 'screen_space_text.inputs:execIn'),
    #                 ('constant_double2.inputs:value', 'screen_space_text.inputs:position'),
    #                 ('constant_float.inputs:value', 'screen_space_text.inputs:size'),
    #                 ('append_string_06.outputs:value', 'screen_space_text.inputs:text'),
    #                 ('attribute_name.inputs:value', 'read_recycling_score.inputs:name'),
    #                 ('attribute_name.inputs:value', 'read_trash_pose_estimation_score.inputs:name'),
    #                 ('read_recycling_score.outputs:value', 'tostr1.inputs:value'),
    #                 ('read_trash_pose_estimation_score.outputs:value', 'tostr2.inputs:value'),
    #                 ('tostr1.outputs:converted', 'appendstr1.inputs:suffix'),
    #                 ('recycling_score_str.inputs:value', 'appendstr1.inputs:value'),
    #                 ('tostr2.outputs:converted', 'appendstr2.inputs:suffix'),
    #                 ('pose_estimation_score_str.inputs:value', 'appendstr2.inputs:value'),
    #                 ('appendstr2.outputs:value', 'appendstr3.inputs:suffix'),
    #                 ('appendstr1.outputs:value', 'appendstr3.inputs:value'),
    #                 ('to_int.outputs:converted', 'to_string.inputs:value'),
    #                 ('constant_string.inputs:value', 'append_string.inputs:a'),
    #                 ('to_string.outputs:converted', 'append_string.inputs:b'),
    #                 ('append_string_03.outputs:value', 'append_string_01.inputs:a'),
    #                 ('appendstr3.outputs:value', 'append_string_01.inputs:b'),
    #                 ('on_playback_tick.outputs:time', 'to_int.inputs:value'),
    #                 ('trashesReadAttrib.outputs:value', 'to_string_01.inputs:value'),
    #                 ('constant_string_01.inputs:value', 'append_string_02.inputs:a'),
    #                 ('to_string_01.outputs:converted', 'append_string_02.inputs:b'),
    #                 ('append_string.outputs:value', 'append_string_03.inputs:a'),
    #                 ('append_string_04.outputs:value', 'append_string_03.inputs:b'),
    #                 ('append_string_02.outputs:value', 'append_string_04.inputs:a'),
    #                 ('constant_string_02.inputs:value', 'append_string_04.inputs:b'),
    #                 ('read_trash_pose_estimation_score.outputs:value', 'add.inputs:a'),
    #                 ('read_recycling_score.outputs:value', 'add.inputs:b'),
    #                 ('add.outputs:sum', 'to_string_02.inputs:value'),
    #                 ('constant_string_03.inputs:value', 'append_string_05.inputs:a'),
    #                 ('to_string_02.outputs:converted', 'append_string_05.inputs:b'),
    #                 ('append_string_01.outputs:value', 'append_string_06.inputs:a'),
    #                 ('append_string_05.outputs:value', 'append_string_06.inputs:b'),
    #             ],

    #             og.Controller.Keys.SET_VALUES: [
    #                 ('screen_space_text.inputs:boxWidth', 0),
    #                 ('attribute_name.inputs:value', 'score'),
    #                 ('recycling_score_str.inputs:value', ', Recycling Score: '),
    #                 ('pose_estimation_score_str.inputs:value', ', Trash Pose Estimation Score: '),
    #                 ('constant_double2.inputs:value', Gf.Vec2d(14.000000208616257, 6.800000101327896)),
    #                 ('constant_color4d.inputs:value', Gf.Vec4d(0.0, 0.6099999863654375, 0.17999999597668648, 0.549999987706542)),
    #                 ('constant_float.inputs:value', 24.0),
    #                 ('constant_string.inputs:value', 'Time: '),
    #                 ('trashesReadAttrib.inputs:name', 'amount'),
    #                 ('constant_string_01.inputs:value', ', Remained/Total: '),
    #                 ('constant_string_02.inputs:value', '/8'),
    #                 ('constant_string_03.inputs:value', ', Total Score: '),
    #                 ('read_trash_pose_estimation_score.inputs:prim', [usdrt.Sdf.Path('/Score/trash_pose_estimations')]),
    #                 ('read_recycling_score.inputs:prim', [usdrt.Sdf.Path('/Score/recycling')]),
    #                 ('trashesReadAttrib.inputs:prim', [usdrt.Sdf.Path('/Score/trashes')]),
    #             ],
    #         },
    #     )

    # except Exception as e:
    #     print(e)

    # try:
    #     og.Controller.edit(
    #         {"graph_path": "/recycling_checker", "evaluator_name": "execution"},
    #         {
    #             og.Controller.Keys.CREATE_NODES: [
    #                 ("on_trigger", "omni.physx.graph.OnTriggerCollider"),
    #                 ("set_prim_active", "omni.graph.action.SetPrimActive"),
    #                 ("increment", "omni.graph.nodes.Increment"),
    #                 ("read_prim_attribute", "omni.graph.nodes.ReadPrimAttribute"),
    #                 ("remainingWriteAttrib", "omni.graph.nodes.WritePrimAttribute"),
    #                 ("trashesWriteAttrib", "omni.graph.nodes.WritePrimAttribute"),
    #                 ("recycling_checker", "aisl.omnigraph.extension.recycling_checker"),
    #             ],
    #             og.Controller.Keys.CONNECT: [
    #                 ("recycling_checker.outputs:execOut", "set_prim_active.inputs:execIn"),
    #                 (
    #                     "recycling_checker.outputs:deactivate_prim_path",
    #                     "set_prim_active.inputs:prim",
    #                 ),
    #                 (
    #                     "recycling_checker.outputs:recycling_points",
    #                     "increment.inputs:increment",
    #                 ),
    #                 ("read_prim_attribute.outputs:value", "increment.inputs:value"),
    #                 (
    #                     "recycling_checker.outputs:execOut",
    #                     "remainingWriteAttrib.inputs:execIn",
    #                 ),
    #                 (
    #                     "recycling_checker.outputs:remained_trashes",
    #                     "remainingWriteAttrib.inputs:value",
    #                 ),
    #                 (
    #                     "recycling_checker.outputs:execOut",
    #                     "trashesWriteAttrib.inputs:execIn",
    #                 ),
    #                 ("increment.outputs:result", "trashesWriteAttrib.inputs:value"),
    #                 ("on_trigger.outputs:enterExecOut", "recycling_checker.inputs:execIn"),
    #                 ("on_trigger.outputs:otherBody", "recycling_checker.inputs:other_body"),
    #                 (
    #                     "on_trigger.outputs:triggerCollider",
    #                     "recycling_checker.inputs:trigger_body",
    #                 ),
    #             ],
    #             og.Controller.Keys.SET_VALUES: [
    #                 ("set_prim_active.inputs:active", False),
    #                 ("read_prim_attribute.inputs:name", "score"),
    #                 ("remainingWriteAttrib.inputs:name", "amount"),
    #                 ("trashesWriteAttrib.inputs:name", "score"),
    #                 ("trashesWriteAttrib.inputs:prim", [usdrt.Sdf.Path("/Score/recycling")]),
    #                 (    "read_prim_attribute.inputs:prim",
    #                     [usdrt.Sdf.Path("/Score/recycling")],
    #                 ),
    #                 (
    #                     "remainingWriteAttrib.inputs:prim",
    #                     [usdrt.Sdf.Path("/Score/trashes")],
    #                 ),
    #             ],
    #         },
    #     )
    # except Exception as e:
    #     print(e)