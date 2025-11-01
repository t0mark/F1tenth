import numpy as np

# x,y,width_left,width_right 형식의 CSV 파일을 불러옵니다.
def load_csv(file_path):
    data = np.genfromtxt(file_path, delimiter=',', skip_header=0)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3]  # 열 순서: x, y, width_left, width_right

# x,y,yaw,s,kappa 형식의 CSV 파일을 불러옵니다.
def load_csv_with_yaw_kappa(file_path):
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3], data[:, 4], data[:, 5]  # 열 순서: x_ref_m, y_ref_m, psi_racetraj_rad, s_racetraj_m, kappa_racetraj_radpm, vx_racetraj_mps

# waypoints_yaw_kappa의 각 웨이포인트에 대해 waypoints_widths에서 가장 가까운 웨이포인트를 찾습니다.
# 그리고 다음과 같은 열을 가진 새로운 CSV 파일을 생성합니다.
# 열 목록: x_ref_m, y_ref_m, width_left_m, width_right_m, psi_racetraj_rad, s_racetraj_m, kappa_racetraj_radpm
def find_closest_waypoint(x, y, waypoints):
    distances = np.sqrt((waypoints[0] - x) ** 2 + (waypoints[1] - y) ** 2)
    closest_index = np.argmin(distances)
    return closest_index

def create_new_csv(waypoints_widths, waypoints_yaw_kappa, output_file_path):
    x_ref_m = []
    y_ref_m = []
    width_left_m = []
    width_right_m = []
    psi_racetraj_rad = []
    s_racetraj_m = []
    kappa_racetraj_radpm = []
    vx_racetraj_mps = []

    for i in range(len(waypoints_yaw_kappa[0])):
        x, y, yaw, s, kappa = waypoints_yaw_kappa[0][i], waypoints_yaw_kappa[1][i], waypoints_yaw_kappa[2][i], waypoints_yaw_kappa[3][i], waypoints_yaw_kappa[4][i]
        closest_index = find_closest_waypoint(x, y, waypoints_widths)
        x_ref_m.append(x)
        y_ref_m.append(y)
        width_left_m.append(waypoints_widths[2][closest_index])
        width_right_m.append(waypoints_widths[3][closest_index])
        psi_racetraj_rad.append(yaw)
        s_racetraj_m.append(s)
        kappa_racetraj_radpm.append(kappa)
        vx_racetraj_mps.append(waypoints_yaw_kappa[5][i])

    # 새로운 CSV 파일을 저장합니다.
    header = 'x_ref_m,y_ref_m,width_left_m,width_right_m,psi_racetraj_rad,s_racetraj_m,kappa_racetraj_radpm,vx_racetraj_mps'
    data = np.column_stack((x_ref_m, y_ref_m, width_left_m, width_right_m, psi_racetraj_rad, s_racetraj_m, kappa_racetraj_radpm, vx_racetraj_mps))
    np.savetxt(output_file_path, data, delimiter='; ', header=header, comments='', fmt='%.4f')


if __name__ == "__main__":
    waypoints_widths = load_csv('/home/vaithak/Downloads/UPenn/F1Tenth/race3_map_edited/width_waypoints-prak.csv')
    waypoints_yaw_kappa = load_csv_with_yaw_kappa('/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/icra_race_2_interactive_fitted.csv')
    # 웨이포인트와 폭 정보를 포함하는 새로운 CSV 파일을 생성합니다.
    create_new_csv(waypoints_widths, waypoints_yaw_kappa, '/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/icra_race_2_interactive_width.csv')
