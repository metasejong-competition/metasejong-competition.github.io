import omni.graph.core as og
import random

# 전역 딕셔너리: 각 아이템은 'type'과 1~10 사이의 랜덤 점수를 가짐.
item_scores = {
    "tissue": {"type": "paper", "score": random.randint(1, 10)},
    "juice": {"type": "plastic", "score": random.randint(1, 10)},
    "disposable_cup": {"type": "plastic", "score": random.randint(1, 10)},
    "wood_block": {"type": "none", "score": random.randint(1, 10)},
    "mug_cup": {"type": "none", "score": random.randint(1, 10)},
    "snack_box": {"type": "paper", "score": random.randint(1, 10)},
    "cola_can": {"type": "aluminum", "score": random.randint(1, 10)},
    "food_can": {"type": "aluminum", "score": random.randint(1, 10)},
}

# 원래 모든 물체 이름을 담은 리스트 (재시작 시 이 값으로 복원됨)
object_list_original = list(item_scores.keys())
object_list = object_list_original.copy()


class OgnRecyclingChecker:
    @staticmethod
    def setup(db: og.Database) -> bool:
        # 노드가 리셋될 때마다 객체 리스트를 원상복구
        global object_list
        object_list = object_list_original.copy()
        return True

    @staticmethod
    def compute(db: og.Database) -> bool:
        try:
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            # 입력된 문자열 값 그대로 사용 (이미 str 타입임)
            trigger_body = db.inputs.trigger_body
            other_body = db.inputs.other_body

            # '/'를 기준으로 경로를 분리하여 마지막 세그먼트 추출
            trigger_last = trigger_body.split("/")[-1] if trigger_body else ""
            other_last = other_body.split("/")[-1] if other_body else ""

            # trigger_body의 마지막 부분이 "trigger_" 접두사를 가지고 있으면 재질(material) 추출,
            # 없으면 전체 문자열을 사용
            if trigger_last.startswith("trigger_"):
                trigger_material = trigger_last[len("trigger_"):]
            else:
                db.log_warning("trigger_body가 예상하는 'trigger_' 접두사를 포함하지 않음. 전체 문자열을 material로 사용합니다.")
                trigger_material = trigger_last

            # 기본 출력값 초기화
            db.outputs.recycling_points = 0.0
            db.outputs.deactivate_prim_path = ""

            # other_body의 마지막 세그먼트가 item_scores의 키와 정확히 일치하는지 확인
            matched_key = None
            if other_last in item_scores:
                matched_key = other_last
            else:
                # 정확히 일치하지 않으면, 전체 other_body 문자열에서 키가 포함되어 있는지 검색
                for key in item_scores.keys():
                    if key in other_body:
                        matched_key = key
                        break

            if matched_key:
                item_info = item_scores[matched_key]
                # 재질 비교: 아이템의 type과 trigger_material이 같으면 점수 부여, 다르면 0점
                if item_info["type"] == trigger_material:
                    db.outputs.recycling_points = item_info["score"]
                else:
                    db.outputs.recycling_points = 0.0

                # other_body 경로에 'trash'가 포함되어 있으면 deactivate_prim_path에 경로 설정
                if "trash" in other_body:
                    db.outputs.deactivate_prim_path = other_body
                else:
                    db.outputs.deactivate_prim_path = ""

                # 전역 리스트(object_list)에서 해당 키를 제거 (이미 제거되었다면 무시)
                global object_list
                if matched_key in object_list:
                    object_list.remove(matched_key)
            else:
                db.log_warning("other_body의 마지막 부분이 정의된 아이템 키와 일치하지 않음")
                db.outputs.recycling_points = 0.0
                db.outputs.deactivate_prim_path = ""

            # 남은 물체 수를 remained_trashes에 출력
            db.outputs.remained_trashes = len(object_list)

        except Exception as error:
            db.log_error(str(error))
            return False

        return True
