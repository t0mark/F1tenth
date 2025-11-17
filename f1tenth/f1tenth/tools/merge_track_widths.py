import numpy as np
import os
import matplotlib.pyplot as plt

# x,y,width_left,width_right 형식의 CSV 파일을 불러옵니다.
def load_csv_widths(file_path):
    # 헤더가 있을 수 있으므로 skip_header=1 추가
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3]  # 열 순서: x, y, width_left, width_right

# x,y,yaw,s,kappa,vx 형식의 CSV 파일을 불러옵니다.
def load_csv_fitted(file_path):
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3], data[:, 4], data[:, 5]  # 열 순서: x, y, yaw, s, kappa, vx

# waypoints_fitted의 각 웨이포인트에 대해 waypoints_widths에서 가장 가까운 웨이포인트를 찾습니다.
def find_closest_waypoint(x, y, waypoints_widths_x, waypoints_widths_y):
    distances = np.sqrt((waypoints_widths_x - x) ** 2 + (waypoints_widths_y - y) ** 2)
    closest_index = np.argmin(distances)
    return closest_index

def create_new_csv(waypoints_widths, waypoints_fitted):
    x_ref_m, y_ref_m, psi_racetraj_rad, s_racetraj_m, kappa_racetraj_radpm, vx_racetraj_mps = waypoints_fitted
    widths_x, widths_y, widths_left, widths_right = waypoints_widths

    merged_data = []

    for i in range(len(x_ref_m)):
        x, y = x_ref_m[i], y_ref_m[i]
        closest_index = find_closest_waypoint(x, y, widths_x, widths_y)
        
        # 병합된 데이터 행 생성
        merged_row = [
            x, y,
            widths_left[closest_index],
            widths_right[closest_index],
            psi_racetraj_rad[i],
            s_racetraj_m[i],
            kappa_racetraj_radpm[i],
            vx_racetraj_mps[i]
        ]
        merged_data.append(merged_row)

    return np.array(merged_data)

def visualize_final_path(waypoints, output_png_path):
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    width_left = waypoints[:, 2]
    width_right = waypoints[:, 3]
    yaw = waypoints[:, 4]
    speeds = waypoints[:, 7]

    # 좌우 경계점 계산
    left_x = x - width_left * np.sin(yaw)
    left_y = y + width_left * np.cos(yaw)
    right_x = x + width_right * np.sin(yaw)
    right_y = y - width_right * np.cos(yaw)

    plt.figure(figsize=(10, 8))
    
    # 속도 기반 컬러맵 설정
    min_speed = np.min(speeds)
    max_speed = np.max(speeds)
    norm_speed = (speeds - min_speed) / (max_speed - min_speed) if (max_speed - min_speed) > 0 else np.zeros_like(speeds)
    colormap = plt.get_cmap('viridis')

    # 경로를 속도에 따라 색칠하여 플로팅
    for i in range(len(x) - 1):
        plt.plot(x[i:i+2], y[i:i+2], color=colormap(norm_speed[i]), linewidth=3)

    # 좌우 경계선 플로팅
    plt.scatter(left_x, left_y, label='Left Boundary', color='lightgreen', s=1)
    plt.scatter(right_x, right_y, label='Right Boundary', color='lightcoral', s=1)
    
    # 컬러바 추가
    sm = plt.cm.ScalarMappable(cmap=colormap, norm=plt.Normalize(vmin=min_speed, vmax=max_speed))
    sm.set_array([])
    cbar = plt.colorbar(sm, label='Speed (m/s)')

    plt.title('Final Waypoints with Widths and Speed Profile')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.savefig(output_png_path)
    print(f"최종 경로 시각화가 다음 경로에 PNG 파일로 저장되었습니다: {output_png_path}")
    plt.close()

def main():
    # 입력 파일 경로 설정
    widths_path = os.path.join('src/f1tenth/data', 'width_log.csv')
    fitted_path = os.path.join('src/f1tenth/data', 'fitted_waypoints.csv')

    # 데이터 로드
    waypoints_widths = load_csv_widths(widths_path)
    waypoints_fitted = load_csv_fitted(fitted_path)

    # 데이터 병합
    final_data = create_new_csv(waypoints_widths, waypoints_fitted)

    # 최종 CSV 파일 저장
    save_path = os.path.join('src/f1tenth/data', 'final_waypoints.csv')
    header = 'x_ref_m;y_ref_m;width_left_m;width_right_m;psi_racetraj_rad;s_racetraj_m;kappa_racetraj_radpm;vx_racetraj_mps'
    np.savetxt(save_path, final_data, delimiter='; ', header=header, comments='', fmt='%.4f')
    print(f"병합된 최종 웨이포인트가 다음 경로에 저장되었습니다: {save_path}")

    # 최종 경로 시각화 및 저장
    viz_save_path = os.path.join('src/f1tenth/data', 'final_waypoints_visualization.png')
    visualize_final_path(final_data, viz_save_path)

if __name__ == "__main__":
    main()
