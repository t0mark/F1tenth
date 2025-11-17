#!/usr/bin/env python3

import numpy as np
import os
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt


def optimize_racing_line(points, target_spacing, wheel_base, max_steering_angle):
    """
    차량의 기구학적 특성을 고려하여 최적 레이싱 라인 생성

    Parameters:
    -----------
    points : np.ndarray
        원본 체크포인트 (x, y)
    target_spacing : float
        목표 포인트 간격 (m)
    wheel_base : float
        차축 거리 (m)
    max_steering_angle : float
        최대 조향각 (rad)

    Returns:
    --------
    optimized_points : np.ndarray
        최적화된 레이싱 라인 포인트 (x, y)
    """
    # 1. 폐루프 완성
    if np.linalg.norm(points[0] - points[-1]) > 0.1:
        points = np.vstack([points, points[0:1]])

    # 2. 누적 거리 계산
    distances = np.zeros(len(points))
    for i in range(1, len(points)):
        distances[i] = distances[i-1] + np.linalg.norm(points[i] - points[i-1])
    total_distance = distances[-1]

    # 3. Cubic spline 생성 (폐루프)
    cs_x = CubicSpline(distances, points[:, 0], bc_type='periodic')
    cs_y = CubicSpline(distances, points[:, 1], bc_type='periodic')

    # 4. 균등 간격으로 포인트 재배치
    num_points = max(int(np.round(total_distance / target_spacing)), len(points))
    uniform_distances = np.linspace(0, total_distance, num_points, endpoint=False)

    optimized_x = cs_x(uniform_distances)
    optimized_y = cs_y(uniform_distances)
    optimized_points = np.column_stack((optimized_x, optimized_y))

    return optimized_points


def visualize_racing_line(original_points, optimized_points, output_png_path):
    """원본 포인트와 최적화된 레이싱 라인 비교 시각화"""
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # (1) 원본 포인트 분포
    axes[0].plot(original_points[:, 0], original_points[:, 1], 'b.-',
                label='원본 체크포인트', markersize=8, linewidth=2)
    axes[0].plot(original_points[0, 0], original_points[0, 1], 'go',
                markersize=12, label='시작점', zorder=5)
    axes[0].set_xlabel('X (m)', fontsize=12)
    axes[0].set_ylabel('Y (m)', fontsize=12)
    axes[0].set_title(f'원본 체크포인트 ({len(original_points)}개)', fontsize=14)
    axes[0].legend(fontsize=10)
    axes[0].axis('equal')
    axes[0].grid(True, alpha=0.3)

    # (2) 최적화된 레이싱 라인
    axes[1].plot(optimized_points[:, 0], optimized_points[:, 1], 'r.-',
                label='최적화된 레이싱 라인', markersize=3, linewidth=2)
    axes[1].plot(optimized_points[0, 0], optimized_points[0, 1], 'go',
                markersize=12, label='시작점', zorder=5)
    # 폐루프 표시
    axes[1].plot([optimized_points[-1, 0], optimized_points[0, 0]],
                [optimized_points[-1, 1], optimized_points[0, 1]],
                'g--', alpha=0.5, linewidth=2, label='폐루프 연결')
    axes[1].set_xlabel('X (m)', fontsize=12)
    axes[1].set_ylabel('Y (m)', fontsize=12)
    axes[1].set_title(f'최적화된 레이싱 라인 ({len(optimized_points)}개)', fontsize=14)
    axes[1].legend(fontsize=10)
    axes[1].axis('equal')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"시각화가 다음 경로에 저장되었습니다: {output_png_path}")

def main():
    """
    pre_checkpoints.csv를 읽어서 최적 레이싱 라인을 생성하고
    checkpoints.csv와 시각화 PNG로 저장
    """
    # ============ 차량 파라미터 설정 ============
    WHEEL_BASE = 0.33  # 차축 거리 (m)
    MAX_STEERING_ANGLE = 0.42  # 최대 조향각 (rad)
    POINT_SPACING = 0.25  # 목표 포인트 간격 (m)

    # 차량의 기구학적 한계 출력
    min_turning_radius = WHEEL_BASE / np.tan(MAX_STEERING_ANGLE)
    max_kappa = 1.0 / min_turning_radius
    print("=" * 60)
    print("차량 파라미터:")
    print(f"  - 차축 거리: {WHEEL_BASE} m")
    print(f"  - 최대 조향각: {MAX_STEERING_ANGLE} rad ({np.degrees(MAX_STEERING_ANGLE):.2f}°)")
    print(f"  - 최소 회전 반경: {min_turning_radius:.3f} m")
    print(f"  - 최대 곡률: {max_kappa:.3f} rad/m")
    print(f"  - 목표 포인트 간격: {POINT_SPACING} m")
    print("=" * 60)

    # 입력 파일 경로 (pre_checkpoints.csv)
    input_file = os.path.join('src/f1tenth/data', 'pre_checkpoints.csv')

    # 파일 존재 여부 확인
    if not os.path.exists(input_file):
        print(f"오류: 입력 파일이 발견되지 않았습니다: {input_file}")
        print("먼저 checkpoint_recorder 노드를 실행하여 pre_checkpoints.csv를 생성하세요.")
        return

    # 체크포인트 로드
    print(f"\n체크포인트 로드 중: {input_file}")
    points = np.genfromtxt(input_file, delimiter=',', skip_header=1)
    print(f"원본 포인트 수: {len(points)}개")

    # 최적 레이싱 라인 생성
    print("\n최적 레이싱 라인 생성 중...")
    optimized_points = optimize_racing_line(
        points,
        target_spacing=POINT_SPACING,
        wheel_base=WHEEL_BASE,
        max_steering_angle=MAX_STEERING_ANGLE
    )

    # 출력 디렉토리 설정
    output_dir = os.path.join('src/f1tenth/data')
    os.makedirs(output_dir, exist_ok=True)

    # checkpoints.csv 저장
    output_csv = os.path.join(output_dir, 'checkpoints.csv')
    np.savetxt(output_csv, optimized_points, delimiter=',', header='x,y', comments='', fmt='%.4f')
    print(f"\n✓ 최적화된 레이싱 라인 저장: {output_csv}")

    # 시각화 저장
    output_png = os.path.join(output_dir, 'checkpoints_visualization.png')
    visualize_racing_line(points, optimized_points, output_png)
    print(f"✓ 시각화 저장: {output_png}")

    # 통계 출력
    total_distance = np.sum([np.linalg.norm(optimized_points[i] - optimized_points[i-1])
                            for i in range(1, len(optimized_points))])
    print("\n" + "=" * 60)
    print(f"원본 포인트 수: {len(points)}개")
    print(f"최적화된 포인트 수: {len(optimized_points)}개")
    print(f"전체 경로 길이: {total_distance:.3f} m")
    print(f"포인트 간 평균 간격: {total_distance/len(optimized_points):.4f} m")
    print("=" * 60)
    print("\n레이싱 라인 생성 완료!")

if __name__ == '__main__':
    main()
