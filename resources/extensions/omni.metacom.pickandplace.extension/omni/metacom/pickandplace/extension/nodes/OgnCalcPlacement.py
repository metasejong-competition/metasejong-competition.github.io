import numpy as np
import omni.graph.core as og
import carb

SCALE_EPSILON = 1e-12


def decompose_transform_ignore_scale(matrix_4x4: np.ndarray):
    """
    4x4 변환 행렬에서 순수 회전 행렬 R과 평행이동 벡터 t를 추출한다.
    스케일이 포함되어 있다면 각 축의 스케일을 제거하여 정규화한다.
    """
    if matrix_4x4.shape != (4, 4):
        raise ValueError("입력 행렬은 (4,4) 크기를 가져야 합니다.")

    # 평행이동 벡터 추출
    t = matrix_4x4[:3, 3]

    # 상위 3x3 부분 추출 (회전과 스케일 포함)
    R_with_scale = matrix_4x4[:3, :3].copy()

    # 각 축의 스케일 (컬럼 벡터의 노름)
    scale_x = np.linalg.norm(R_with_scale[:, 0])
    scale_y = np.linalg.norm(R_with_scale[:, 1])
    scale_z = np.linalg.norm(R_with_scale[:, 2])

    if scale_x < SCALE_EPSILON or scale_y < SCALE_EPSILON or scale_z < SCALE_EPSILON:
        raise ValueError(
            "한 축 이상의 스케일이 0 또는 매우 작습니다. (스케일 제거 불가)"
        )

    # 스케일을 제거하여 순수 회전 행렬 R 구성
    R = R_with_scale.copy()
    R[:, 0] /= scale_x
    R[:, 1] /= scale_y
    R[:, 2] /= scale_z

    return R, t


def transform_vector_ignoring_scale(
    matrix_4x4: np.ndarray, vector_3: np.ndarray
) -> np.ndarray:
    """
    4x4 변환 행렬에서 스케일을 무시하고 추출한 (R, t)로
    주어진 3D 벡터에 대해 R * vector + t 연산을 적용한다.
    """
    R, t = decompose_transform_ignore_scale(matrix_4x4)
    return R @ vector_3 + t


class OgnCalcPlacement:
    @staticmethod
    def compute(db: og.Database):
        """
        전치된 형태의 16개 float로 구성된 4x4 행렬(속성: transformation)과
        3D 벡터(속성: translate_vec)를 입력받아 스케일을 무시한 회전/평행이동을 적용한 결과를
        db.outputs.result에 출력한다.
        """
        try:
            # 입력값 읽기
            matrix_input = db.inputs.transformation  # 예: 16개 float 값
            vector_input = db.inputs.translate_vec  # 예: 3개 float 값
            # numpy array로 변환 후, reshape한 뒤 전치하여 올바른 행렬로 복원
            matrix_4x4 = np.array(matrix_input, dtype=float).reshape(4, 4).T
            vector_3 = np.array(vector_input, dtype=float)

            # 변환 계산 (스케일 무시)
            result_vec = transform_vector_ignoring_scale(matrix_4x4, vector_3)

            # 결과를 출력 속성에 저장 (리스트 형태)
            db.outputs.result = result_vec.tolist()

        except Exception as error:
            db.log_error(str(error))
            return False

        return True
