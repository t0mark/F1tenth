import enum
from .utils import find_closest_index

class StateType(enum.Enum):
    GB_TRACK = 'GB_TRACK' 
    TRAILING = 'TRAILING' 
    OVERTAKE = 'OVERTAKE'

def string_to_state_type(state_string: str) -> StateType:
    """
    문자열을 StateType 열거형으로 변환합니다.
    """
    try:
        return StateType[state_string]
    except KeyError:
        return None
    
def state_type_to_string(state_type: StateType) -> str:
    """
    StateType 열거형을 문자열로 변환합니다.
    """
    if isinstance(state_type, StateType):
        return state_type.value
    else:
        raise ValueError(f"Invalid state type: {state_type}")

def DefaultStateLogic(state_machine):
    """
    다른 상태들을 포괄하는 전체 상태 로직입니다.
    """ 
    # 위 조건을 if 문으로 표현합니다.
    if state_machine.state == StateType.GB_TRACK:
        return GlobalTracking(state_machine)
    elif state_machine.state == StateType.TRAILING:
        return Trailing(state_machine)
    elif state_machine.state == StateType.OVERTAKE:
        return Overtaking(state_machine)
    else:
        raise NotImplementedError(f"State {state_machine.state} not recognized")

"""
각 상태에서의 동작을 정의합니다.
모든 함수는 간결하게 작성하고 배열을 반환해야 합니다.
"""
def GlobalTracking(state_machine):
    curr_s = state_machine.car_s
    s_idx = find_closest_index(state_machine.wpnts_s_array, curr_s)
    return [state_machine.glb_wpnts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]

def Trailing(state_machine):
    # 필요하면 마지막으로 유효했던 스플라인을 따라갈 수 있도록 합니다.
    curr_s = state_machine.car_s
    s_idx = find_closest_index(state_machine.wpnts_s_array, curr_s)
    if state_machine.last_valid_avoidance_wpnts is not None:
        spline_wpts = state_machine.get_spline_wpts()
        return [spline_wpts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]
    else:
        return [state_machine.glb_wpnts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]

def Overtaking(state_machine):
    spline_wpts = state_machine.get_spline_wpts()
    s_idx = find_closest_index(state_machine.wpnts_s_array, state_machine.car_s)
    return [spline_wpts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]


# ------------------ 상태 전환 ----------------------
def dummy_transition(state_machine)->str:
    return StateType.GB_TRACK
        
def timetrials_transition(state_machine)->str:
    return StateType.GB_TRACK

def head_to_head_transition(state_machine)->str:
    if state_machine.state == StateType.GB_TRACK:
        return SplineGlobalTrackingTransition(state_machine)
    elif state_machine.state == StateType.TRAILING:
        return SplineTrailingTransition(state_machine)
    elif state_machine.state == StateType.OVERTAKE:
        return SplineOvertakingTransition(state_machine)
    else:
        raise NotImplementedError(f"State {state_machine.state} not recognized")


def SplineGlobalTrackingTransition(state_machine) -> StateType:
    """`StateType.GB_TRACK` 상태에서의 전이 조건.

    개선된 로직:
    - 정적 장애물 발견: 즉시 추월 기회 확인
    - 동적 장애물 발견: TRAILING으로 전이 후 상황 평가
    """
    if state_machine._check_gbfree:
        return StateType.GB_TRACK

    # 정적 장애물이 있고 추월 가능하면 바로 OVERTAKE
    if state_machine._check_static_obstacle_ahead:
        if (state_machine._check_ot_sector and
            state_machine._check_availability_spline_wpts and
            state_machine._check_ofree):
            return StateType.OVERTAKE

    # 그 외의 경우 TRAILING으로 전이
    return StateType.TRAILING


def SplineTrailingTransition(state_machine) -> StateType:
    """`StateType.TRAILING` 상태에서의 전이 조건.

    개선된 로직:
    - GB_FREE이면 레이스라인 복귀
    - 추월 필요성 판단: 정적 장애물 또는 느린 동적 장애물
    - 추월 필요하고 안전하면 OVERTAKE로 전이
    - 그 외에는 안전거리 유지하며 TRAILING
    """
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    # GB가 비어있고 레이스라인에 가까우면 복귀
    if gb_free and state_machine._check_close_to_raceline:
        return StateType.GB_TRACK

    # 추월 필요성 판단 (정적 또는 느린 동적 장애물)
    need_overtake = state_machine._check_need_overtake

    # 추월이 필요하고 안전한 추월 경로가 있으면 OVERTAKE
    if (need_overtake and
        ot_sector and
        state_machine._check_availability_spline_wpts and
        state_machine._check_ofree):
        return StateType.OVERTAKE

    # 그 외에는 TRAILING 유지 (안전거리 유지)
    return StateType.TRAILING


def SplineOvertakingTransition(state_machine) -> StateType:
    """`StateType.OVERTAKE` 상태에서의 전이 조건."""
    in_ot_sector = state_machine._check_ot_sector
    spline_valid = state_machine._check_availability_spline_wpts
    o_free = state_machine._check_ofree

    # 스플라인 경로가 장애물 위에 놓이면 추종 상태로 전환합니다.
    if not o_free:
        return StateType.TRAILING
    if in_ot_sector and o_free and spline_valid:
        return StateType.OVERTAKE
    # 추월 중 스플라인이 무효가 되면 추종 상태로 돌아갑니다.
    elif in_ot_sector and not spline_valid and not o_free:
        return StateType.TRAILING
    # 추월 구간이 아니고 GB가 비어 있으면 GB_TRACK으로 전환합니다.
    elif not in_ot_sector and state_machine._check_gbfree:
        return StateType.GB_TRACK
    # 추월 구간이 아니고 GB가 비어 있지 않으면 Trailing으로 전환합니다.
    else:
        return StateType.TRAILING
