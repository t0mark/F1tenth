#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA
import os # 파일 경로 조작을 위한 임포트
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt # 플로팅 및 PNG 저장을 위한 임포트

def prune_points(points, distance_threshold=0.15):
    """
    점 간 거리가 `distance_threshold` 이상인 점만 남기도록 정리합니다.
    """
    pruned_points = [points[0]]
    for i in range(1, len(points)):
        dist = np.linalg.norm(points[i] - pruned_points[-1])
        if dist >= distance_threshold:
            pruned_points.append(points[i])
    return np.array(pruned_points)

def calculate_smooth_speed_profile(kappas, max_speed=6.0, min_speed=1.0, sigma=2):
    """
    곡률 기반으로 속도 계산 후 가우시안 필터로 스무딩
    """
    if len(kappas) == 0 or np.all(kappas == 0):
        return np.full(len(kappas), max_speed)
        
    max_kappa = np.max(np.abs(kappas))
    
    if max_kappa == 0:
        return np.full(len(kappas), max_speed)

    speeds = []
    for kappa in kappas:
        normalized_kappa = np.abs(kappa) / max_kappa
        speed = max_speed - (normalized_kappa * (max_speed - min_speed))
        speeds.append(max(min_speed, speed))
    
    smoothed_speeds = gaussian_filter1d(speeds, sigma=sigma)
    
    return np.array(smoothed_speeds)

def FitPath(points):
    """
    (x, y) 점에 3차 스플라인을 맞추고, yaw, 곡률, 누적 거리, 속도를 계산합니다.
    """
    x = points[:, 0]
    y = points[:, 1]
    t = np.linspace(0, 1, len(x))
    cs_x = CubicSpline(t, x, bc_type='clamped')
    cs_y = CubicSpline(t, y, bc_type='clamped')
    t_new = np.linspace(0, 1, len(x))
    x_new = cs_x(t_new)
    y_new = cs_y(t_new)

    dx = np.gradient(x_new)
    dy = np.gradient(y_new)
    yaw_new = np.arctan2(dy, dx)

    kappa = np.zeros_like(x_new)
    for i in range(1, len(x_new) - 1):
        dx = x_new[i + 1] - x_new[i - 1]
        dy = y_new[i + 1] - y_new[i - 1]
        d2x = (x_new[i + 1] - 2 * x_new[i] + x_new[i - 1])
        d2y = (y_new[i + 1] - 2 * y_new[i] + y_new[i - 1])
        denominator = (dx ** 2 + dy ** 2) ** (3 / 2)
        if denominator > 1e-6:
            kappa[i] = (dx * d2y - dy * d2x) / denominator
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]

    s = np.zeros_like(x_new)
    for i in range(1, len(x_new)):
        s[i] = s[i - 1] + np.sqrt((x_new[i] - x_new[i - 1]) ** 2 + (y_new[i] - y_new[i - 1]) ** 2)

    vx = calculate_smooth_speed_profile(kappa)

    return np.column_stack((x_new, y_new, yaw_new, s, kappa, vx))

def visualize_path_to_png(waypoints, output_png_path):
    plt.figure(figsize=(10, 8))
    
    # 속도 값을 색상 매핑에 사용합니다.
    speeds = waypoints[:, 5] # vx_racetraj_mps 컬럼
    min_speed = np.min(speeds)
    max_speed = np.max(speeds)
    
    # 속도를 [0, 1] 범위로 정규화합니다.
    norm_speed = (speeds - min_speed) / (max_speed - min_speed) if (max_speed - min_speed) > 0 else np.zeros_like(speeds)
    
    # 컬러맵을 생성합니다.
    colormap = plt.get_cmap('viridis')
    
    # 각 세그먼트를 해당 색상으로 플로팅합니다.
    for i in range(len(waypoints) - 1):
        plt.plot(waypoints[i:i+2, 0], waypoints[i:i+2, 1], color=colormap(norm_speed[i]), linewidth=2)
    
    # 웨이포인트를 산점도로 표시하고 컬러바를 추가합니다.
    plt.scatter(waypoints[:, 0], waypoints[:, 1], c=speeds, cmap='viridis', s=20, zorder=2)
    plt.colorbar(label='Speed (m/s)')
    plt.title('Fitted Waypoints Speed Heatmap')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True)
    plt.axis('equal') # x, y 축 스케일을 동일하게 유지
    
    plt.savefig(output_png_path)
    print(f"경로 시각화가 다음 경로에 PNG 파일로 저장되었습니다: {output_png_path}")
    plt.close() # 플롯을 닫아 메모리 해제

def main():
    # 입력 웨이포인트 파일 경로 (예: data/checkpoints.csv)
    # 스크립트가 실행되는 위치를 기준으로 상대 경로를 사용합니다.
    input_waypoint_file = os.path.join('src/path_planner/path_planner/data', 'checkpoints.csv')
    
    # 파일 존재 여부 확인
    if not os.path.exists(input_waypoint_file):
        print(f"오류: 입력 웨이포인트 파일이 다음 경로에서 발견되지 않았습니다: {input_waypoint_file}")
        print("스크립트를 실행하는 위치를 기준으로 'data' 폴더 안에 'checkpoints.csv' 파일이 있는지 확인하거나, 절대 경로를 제공하십시오.")
        return

    # 웨이포인트 로드
    # CSV 파일은 x, y 좌표만 포함한다고 가정합니다.
    waypoints = np.genfromtxt(input_waypoint_file, delimiter=',', skip_header=1)
    
    # 닫힌 루프 트랙을 위해 첫 번째 점을 배열의 끝에 추가합니다.
    waypoints = np.append(waypoints, [waypoints[0]], axis=0)

    print(f"원본 웨이포인트 개수: {len(waypoints)}")

    # 웨이포인트에 곡선을 맞추고 속성 계산
    fitted_waypoints = FitPath(waypoints)

    # 결과 CSV 파일 저장 경로 (예: src/path_planner/data/fitted_waypoints.csv)
    output_csv_path = os.path.join('src/path_planner/path_planner/data', 'fitted_waypoints.csv')

    # CSV 파일 헤더 정의
    header = 'x_ref_m,y_ref_m,psi_racetraj_rad,s_racetraj_m,kappa_racetraj_radpm,vx_racetraj_mps'
    
    # CSV 파일로 저장
    np.savetxt(output_csv_path, fitted_waypoints, delimiter=',', header=header, comments='', fmt='%.4f')
    print(f"가공된 웨이포인트가 다음 경로에 저장되었습니다: {output_csv_path}")

    # 경로 시각화를 PNG 파일로 저장
    output_png_path = os.path.join('src/path_planner/path_planner/data', 'fitted_waypoints_speed_heatmap.png')
    visualize_path_to_png(fitted_waypoints, output_png_path)

if __name__ == '__main__':
    main()
