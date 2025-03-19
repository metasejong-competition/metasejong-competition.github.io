from pxr import Gf
import numpy as np
from scipy.spatial.transform import Rotation as R
from pxr import UsdPhysics, Sdf
import omni.kit.commands
import omni.usd
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.prims import RigidPrim
from omni.kit.commands import execute
from omni.physx.scripts import utils
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats


def set_friction():
    execute(
        "AddRigidBodyMaterialCommand",
        stage=omni.usd.get_context().get_stage(),
        path="/world/ObjectFriction",
    )
    execute(
        "ChangeProperty",
        prop_path=Sdf.Path("/world/ObjectFriction.physics:dynamicFriction"),
        value=0.4,
        prev=0.0,
        usd_context_name=omni.usd.get_context().get_stage(),
    )
    execute(
        "ChangeProperty",
        prop_path=Sdf.Path("/world/ObjectFriction.physics:staticFriction"),
        value=0.4,
        prev=0.0,
        usd_context_name=omni.usd.get_context().get_stage(),
    )


def publish_objects(asset_attribute, ground_truth_output_dir, stage, robot_start_point, standard_pos, object_list):
    ## gt 생성용 ##
    import os
    output_file = os.path.join(ground_truth_output_dir, "publish_issacsim.txt")

    if not os.path.exists(output_file):
        open(output_file, "w").close()
    else:
        os.remove(output_file)

    prim_paths = []
    offset_range = 3

    with open(output_file, "w") as f:
        f.write(f"StartPoint,{robot_start_point[0]},{robot_start_point[1]},{robot_start_point[2]}\n")

        for building in standard_pos:
            for obj in object_list:
                prim_path = f"/world/"+building+"_"+obj
                asset = asset_attribute[obj]
                # stage.add_reference_to_stage(
                #     os.path.join(asset_attribute[obj]["usd_dir"], asset_attribute[obj]["usd_file"]),
                #     prim_path
                # )
                usd_stage = stage.get_current_stage()
                # RigidPrim(prim_path, mass=1.0)
                # prim = usd_stage.GetPrimAtPath(prim_path)
                # UsdPhysics.CollisionAPI.Apply(prim)

                # rand_obj_trans = standard_pos[building]["translation"] + np.random.uniform(-offset_range, offset_range, size=3)

                # obj_trans = Gf.Vec3d(rand_obj_trans[0], rand_obj_trans[1], standard_pos[building]["translation"][2])
                # obj_quat = R.from_euler('zyx', standard_pos[building]["orientation"], degrees=True).as_quat()
                # obj_orient = Gf.Quatd(obj_quat[3], obj_quat[0], obj_quat[1], obj_quat[2])
                # obj_scale = Gf.Vec3d(*standard_pos[building]["scale"])

                # prim = get_prim_at_path(prim_path)
                # prim.GetAttribute("xformOp:translate").Set(obj_trans)
                # prim.GetAttribute("xformOp:orient").Set(obj_orient)
                # prim.GetAttribute("xformOp:scale").Set(obj_scale)
                # prim_paths.append((prim_path, standard_pos[building], obj))
                # x, y 방향 각각 -0.5 ~ 0.5 범위의 난수 생성 -> 3으로 변경
                offset_x = np.random.uniform(-3, 3)
                offset_y = np.random.uniform(-3, 3)
                # 베이스 위치에 난수 오프셋 추가 (z는 그대로 유지)
                obj_rand_x = standard_pos[building]["translation"][0] + offset_x
                obj_rand_y = standard_pos[building]["translation"][1] + offset_y
                obj_z = standard_pos[building]["translation"][2]

                new_position = (
                    obj_rand_x,
                    obj_rand_y,
                    obj_z,
                )
                quats_orientation = euler_angles_to_quats(np.array(asset["orientation"]), degrees=True, extrinsic=False)

                execute(
                    "IsaacSimSpawnPrim",
                    usd_path=os.path.join(asset_attribute[obj]["usd_dir"], asset_attribute[obj]["usd_file"]),
                    prim_path=prim_path,
                    translation=new_position,
                    rotation=(
                        quats_orientation[3],
                        quats_orientation[0],
                        quats_orientation[1],
                        quats_orientation[2],
                    ),
                )
                usd_prim = usd_stage.GetPrimAtPath(prim_path)
                # for i in range(100):
                #     print(usd_prim)
                utils.setRigidBody(usd_prim, "convexDecomposition", False)
                label = f"{obj}"
                add_update_semantics(usd_prim, semantic_label=label, type_label="class")
                prim_paths.append((prim_path, standard_pos[building], obj))
                if prim_path.startswith("/world/Chungmu"):
                    if not prim_path.endswith("wood_block"):
                        f.write(f"{prim_path},{obj_rand_x},{obj_rand_y},{obj_z}\n")

    return prim_paths


def set_object_attributes_semantics(asset_attribute, standard_pos, object_list):
    for building in standard_pos:
        for obj in object_list:
            prim_path = f"/world/"+building+"_"+obj
            prim = get_prim_at_path(prim_path)
            execute(
                "IsaacSimScalePrim",
                prim_path=prim_path,
                scale=asset_attribute[obj]["scale"],
            )
            execute(
                "BindMaterial",
                material_path="/world/ObjectFriction",
                prim_path=prim_path,
                strength=["weakerThanDescendants"],
                material_purpose="",
            )
        utils.setRigidBody(prim, "convexDecomposition", False)
        label = f"{obj}"
        add_update_semantics(prim, semantic_label=label, type_label="class")


def set_object_physics(prim_paths):
    for prim_path, _, _ in prim_paths:
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:solverPositionIterationCount"),
            value=60,
            prev=16,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:solverVelocityIterationCount"),
            value=60,
            prev=1,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
        execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.physxRigidBody:maxDepenetrationVelocity"),
            value=20.0,
            prev=3.0,
            usd_context_name=omni.usd.get_context().get_stage(),
        )
