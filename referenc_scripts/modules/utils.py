import numpy as np

# def parse_waypoint(waypoint_arr):
#     """
#     # x_ref_m; y_ref_m; width_right_m; width_left_m; x_normvec_m; y_normvec_m; alpha_m; s_racetraj_m; psi_racetraj_rad; kappa_racetraj_radpm; vx_racetraj_mps; ax_racetraj_mps2
#     Parse the waypoint array into a dictionary.
#     Args:
#         Single waypoint array.
#     """
#     waypoint_dict = {
#         "x_ref_m": waypoint_arr[0],
#         "y_ref_m": waypoint_arr[1],
#         "width_right_m": waypoint_arr[2],
#         "width_left_m": waypoint_arr[3],
#         "x_normvec_m": waypoint_arr[4],
#         "y_normvec_m": waypoint_arr[5],
#         "alpha_m": waypoint_arr[6],
#         "s_racetraj_m": waypoint_arr[7],
#         "psi_racetraj_rad": waypoint_arr[8],
#         "kappa_racetraj_radpm": waypoint_arr[9],
#         "vx_racetraj_mps": waypoint_arr[10],
#         "ax_racetraj_mps2": waypoint_arr[11]
#     }
#     return waypoint_dict

def column_numbers_for_waypoints():
    """
    Returns the column numbers for the waypoints.
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
    Converts the psi angle to the range [-pi, pi].
    """
    new_psi = psi + np.pi / 2

    # Bring the angle to the range [-pi, pi] in a vectorized way
    new_psi[new_psi > np.pi] -= 2 * np.pi

    return new_psi

def find_closest_index(sorted_arr, val):
    """
    Find the index of the closest value in a sorted array.
    Args:
        sorted_arr: Sorted array.
        val: Value to find.
    Returns:
        Index of the closest value.
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
    
sector_to_track_lengths_dict = {
    0: 4.5,
    1: 12.0,
    2: 13.7,
    3: 14.8,
    4: 16.8,
    5: 20.5,
    6: 23.5,
    7: 25.5,
    8: 30.5,
    9: 38.36,
    10: 42.5,
    11: 52.8,
    12: 57.8,
    13: 61.8,
    14: 65.0,
}

sector_to_velocity_map = {
    0: 6.5,
    1: 6.5,
    2: 6.5,
    3: 5.0,
    4: 4.5,
    5: 2.0,
    6: 5.0,
    7: 3.0,
    8: 4.5,
    9: 6.5,
    10: 5.0,
    11: 5.0,
    12: 6.0,
    13: 6.0,
    14: 6.5,
}

sector_to_lookahead_map = {
    0: 6.5,
    1: 3.0,
    2: 4.5,
    3: 2.5,
    4: 1.0,
    5: 1.5,
    6: 1.5,
    7: 1.5,
    8: 2.5,
    9: 2.5,
    10: 1.0,
    11: 2.5,
    12: 3.0,
    13: 1.5,
    14: 6.0,
}
