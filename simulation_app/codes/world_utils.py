import numpy as np
from omni.isaac.core.utils import viewports, stage
from omni.kit.commands import execute
import carb

def setup_world(camera_view_eye, camera_view_target, BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH):
    # 카메라 위치 설정
    carmera_view_eye = np.array(camera_view_eye)
    carmera_view_target = np.array(camera_view_target)
    viewports.set_camera_view(eye=carmera_view_eye, target=carmera_view_target)
    carb.log_info(f"[World Setup] Camera view set: eye={carmera_view_eye}, target={carmera_view_target}")

    # 배경 환경 로드
    stage.add_reference_to_stage(BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
    carb.log_info(f"[World Setup] Background stage loaded from: {BACKGROUND_USD_PATH}")

    # 특정 Prim 스케일 조정
    # execute(
    #     "IsaacSimScalePrim",
    #     prim_path="/World/metasejong",
    #     scale=(0.01, 0.01, 0.01),
    # )
    # carb.log_info("[World Setup] /World/metasejong scaled to (0.01, 0.01, 0.01)")
