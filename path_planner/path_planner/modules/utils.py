import numpy as np

def column_numbers_for_waypoints():
    """
    웨이포인트 각 항목의 열 번호를 반환합니다.
    """
    return {
        "x_ref_m": 0,
        "y_ref_m": 1,
        "width_right_m": 2,
        "width_left_m": 3,
        # "x_normvec_m": 4,
        # "y_normvec_m": 5,
        # "alpha_m": 6,
        "s_racetraj_m": 5,
        "psi_racetraj_rad": 4,
        "kappa_racetraj_radpm": 6,
        "vx_racetraj_mps": 7,
        # "ax_racetraj_mps2": 11
    }

def convert_psi(psi):
    """
    psi 각도를 [-pi, pi] 범위로 변환합니다.
    """
    new_psi = psi + np.pi / 2

    # 벡터 연산으로 각도를 [-pi, pi] 범위에 맞춥니다.
    new_psi[new_psi > np.pi] -= 2 * np.pi

    return new_psi

def find_closest_index(sorted_arr, val):
    """
    정렬된 배열에서 가장 가까운 값의 인덱스를 찾습니다.
    Args:
        sorted_arr: 정렬된 배열
        val: 찾고자 하는 값
    Returns:
        가장 가까운 값의 인덱스
    """
    idx = np.searchsorted(sorted_arr, val)
    if idx == 0:
        return 0
    elif idx == len(sorted_arr):
        return len(sorted_arr) - 1
    else:
        left = sorted_arr[idx - 1]
        right = sorted_arr[idx]
        return idx - 1 if abs(val - left) < abs(val - right) else idx

