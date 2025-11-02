import enum
from .utils import find_closest_index
from f1tenth_icra_race_msgs.msg import Wpnt

class StateType(enum.Enum):
    GB_TRACK = 'GB_TRACK' 
    TRAILING = 'TRAILING' 
    OVERTAKE = 'OVERTAKE'
    AVOID_STATIC = 'AVOID_STATIC'

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
    if state_machine.state == StateType.GB_TRACK:
        return GlobalTracking(state_machine)
    elif state_machine.state == StateType.TRAILING:
        return Trailing(state_machine)
    elif state_machine.state == StateType.OVERTAKE:
        return Overtaking(state_machine)
    elif state_machine.state == StateType.AVOID_STATIC:
        return AvoidStatic(state_machine)
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
    """
    TRAILING 상태: 전방 동적 장애물을 추종합니다.

    로직:
    - 동적 장애물이 나보다 빠르거나 비슷하면 → 글로벌 웨이포인트 따라가기
    - 동적 장애물이 나보다 느리면 → 상태 전환 로직이 OVERTAKE로 전환
    """
    curr_s = state_machine.car_s
    s_idx = find_closest_index(state_machine.wpnts_s_array, curr_s)

    # 전방의 가장 가까운 동적 장애물 확인
    closest_dynamic_obs, _ = state_machine._get_closest_dynamic_obstacle()

    if closest_dynamic_obs is not None:
        # 동적 장애물의 속도와 내 속도 비교
        # 장애물이 나보다 빠르거나 비슷하면 (10% 이내) 글로벌 웨이포인트 따라가기
        if closest_dynamic_obs.vs >= state_machine.car_vs * 0.9:
            # 글로벌 웨이포인트를 따라가며 정상 주행
            return [state_machine.glb_wpnts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]

    # 기존 로직: 스플라인이 있으면 따라가고, 없으면 글로벌 웨이포인트
    if state_machine.last_valid_avoidance_wpnts is not None:
        spline_wpts = state_machine.get_spline_wpts()
        return [spline_wpts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]
    else:
        return [state_machine.glb_wpnts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]

def Overtaking(state_machine):
    spline_wpts = state_machine.get_spline_wpts()
    s_idx = find_closest_index(state_machine.wpnts_s_array, state_machine.car_s)
    return [spline_wpts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]

def AvoidStatic(state_machine):
    """
    정적 장애물 회피를 위한 간단한 로직.
    가장 가까운 정적 장애물을 찾아, _more_space를 이용해 회피 방향을 결정하고,
    해당 방향으로 전역 웨이포인트를 평행 이동시켜 간단한 회피 경로를 생성합니다.
    """
    horizon = state_machine.overtaking_horizon_m
    closest_static_obs = None
    min_gap = float('inf')

    # 가장 가까운 정적 장애물 찾기
    for obs in state_machine.obstacles:
        if obs.is_static:
            gap = (obs.s_center - state_machine.car_s) % state_machine.track_length
            if gap < horizon and gap < min_gap:
                closest_static_obs = obs
                min_gap = gap

    if closest_static_obs is None:
        # 정적 장애물이 없으면 GlobalTracking과 동일하게 동작
        curr_s = state_machine.car_s
        s_idx = find_closest_index(state_machine.wpnts_s_array, curr_s)
        return [state_machine.glb_wpnts[(s_idx + i)%state_machine.num_glb_wpnts] for i in range(state_machine.n_loc_wpnts)]

    # 회피 방향 결정
    opp_wpnt_idx = find_closest_index(state_machine.wpnts_s_array, closest_static_obs.s_center)
    side, d_apex = state_machine._more_space(closest_static_obs, opp_wpnt_idx)

    # 간단한 회피 경로 생성 (전역 웨이포인트 평행 이동)
    local_waypoints = []
    curr_s = state_machine.car_s
    s_idx = find_closest_index(state_machine.wpnts_s_array, curr_s)

    for i in range(state_machine.n_loc_wpnts):
        wpnt = state_machine.glb_wpnts[(s_idx + i) % state_machine.num_glb_wpnts]

        # 장애물 주변 영역에서만 d_m 값을 변경
        s_diff = (wpnt.s_m - closest_static_obs.s_center) % state_machine.track_length
        if abs(s_diff) < state_machine.overtaking_horizon_m / 2.0:
            # d_apex 값을 사용하여 횡방향 이동
            shifted_wpnt = Wpnt()
            shifted_wpnt.id = wpnt.id
            shifted_wpnt.x_m, shifted_wpnt.y_m = state_machine.converter.get_cartesian(wpnt.s_m, d_apex)

            # 곡률 기반 속도 프로파일 적용
            # 해당 위치의 곡률을 가져와서 속도를 조정합니다.
            wpnt_idx = find_closest_index(state_machine.wpnts_s_array, wpnt.s_m)
            kappa = abs(state_machine.kappa_array[wpnt_idx])

            # 곡률이 클수록 속도를 낮춤 (급커브에서 안전하게)
            # speed_factor = 1.0 / (1.0 + k * |kappa|), k=5.0 (조정 계수)
            curvature_factor = 1.0 / (1.0 + 5.0 * kappa)
            # 최소 0.6배, 최대 0.9배로 제한
            curvature_factor = max(0.6, min(0.9, curvature_factor))

            shifted_wpnt.vx_mps = wpnt.vx_mps * curvature_factor
            shifted_wpnt.s_m = wpnt.s_m
            shifted_wpnt.d_m = d_apex
            local_waypoints.append(shifted_wpnt)
        else:
            local_waypoints.append(wpnt)

    return local_waypoints


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
    elif state_machine.state == StateType.AVOID_STATIC:
        return SplineAvoidStaticTransition(state_machine)
    else:
        raise NotImplementedError(f"State {state_machine.state} not recognized")


def SplineGlobalTrackingTransition(state_machine) -> StateType:
    """`StateType.GB_TRACK` 상태에서의 전이 조건.

    개선된 로직:
    - 정적 장애물 발견: AVOID_STATIC 또는 OVERTAKE로 전이
    - 동적 장애물 발견: TRAILING으로 전이 후 상황 평가
    """
    if state_machine._check_gbfree:
        return StateType.GB_TRACK

    # 정적 장애물이 있으면
    if state_machine._check_static_obstacle_ahead:
        # 스플라인 기반 추월이 가능하면 OVERTAKE
        if (state_machine._check_ot_sector and
            state_machine._check_availability_spline_wpts and
            state_machine._check_ofree):
            return StateType.OVERTAKE
        # 그렇지 않으면 간단한 정적 회피 상태로 전환
        else:
            return StateType.AVOID_STATIC

    # 동적 장애물이 있으면 TRAILING으로 전이
    if not state_machine._check_gbfree:
        return StateType.TRAILING

    return StateType.GB_TRACK


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

def SplineAvoidStaticTransition(state_machine) -> StateType:
    """`StateType.AVOID_STATIC` 상태에서의 전이 조건."""
    # 정적 장애물 회피가 완료되면 GB_TRACK으로 복귀
    if state_machine._check_gbfree:
        return StateType.GB_TRACK
    
    # 다른 동적 장애물이 나타나면 TRAILING으로 전환
    if not state_machine._check_static_obstacle_ahead and not state_machine._check_gbfree:
        return StateType.TRAILING

    return StateType.AVOID_STATIC

