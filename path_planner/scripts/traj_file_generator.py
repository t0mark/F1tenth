#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy import linalg as LA
from visualization_msgs.msg import MarkerArray, Marker

MODE = 2

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

def FitPath(points):
    """
    점은 (x, y, yaw) 형태로 전달됩니다.
    """
    if MODE == 1:
        return points
    elif MODE == 2:
        # (x, y) 점에 3차 스플라인을 맞춥니다.
        from scipy.interpolate import CubicSpline
        # 반복 지점 수(repetition_points)를 2로 설정합니다.
        # x = points[:-(repetition_points - 1), 0]
        # y = points[:-(repetition_points - 1), 1]
        x = points[:, 0]
        y = points[:, 1]
        t = np.linspace(0, 1, len(x))
        cs_x = CubicSpline(t, x, bc_type='clamped')
        cs_y = CubicSpline(t, y, bc_type='clamped')
        t_new = np.linspace(0, 1, 500)
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)

        # 점 간 거리가 0.2 이상인 경우만 남기도록 정리합니다.
        # x_new, y_new = prune_points(np.column_stack((x_new, y_new)), distance_threshold=0.2).T
        # print("정리 후 점 개수: ", len(x_new))

        # 요 각도는 dy/dx = y'(t) / x'(t)로 계산합니다.
        dx = np.gradient(x_new)
        dy = np.gradient(y_new)
        yaw_new = np.arctan2(dy, dx)
        # yaw_new = np.unwrap(yaw_new)  # 불연속이 생기지 않도록 각도를 전개합니다.

        # 각 지점의 곡률(kappa)을 계산합니다.
        kappa = np.zeros_like(x_new)
        for i in range(1, len(x_new) - 1):
            dx = x_new[i + 1] - x_new[i - 1]
            dy = y_new[i + 1] - y_new[i - 1]
            d2x = (x_new[i + 1] - 2 * x_new[i] + x_new[i - 1])
            d2y = (y_new[i + 1] - 2 * y_new[i] + y_new[i - 1])
            kappa[i] = (dx * d2y - dy * d2x) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]

        # 각 지점에 대한 `s` 값을 계산합니다.
        s = np.zeros_like(x_new)
        for i in range(1, len(x_new)):
            s[i] = s[i - 1] + np.sqrt((x_new[i] - x_new[i - 1]) ** 2 + (y_new[i] - y_new[i - 1]) ** 2)

        vx = np.zeros_like(x_new)
        vx += 2.5 # 현재는 2.5 m/s로 설정되어 있으며, TODO: 개선 필요

        return np.column_stack((x_new, y_new, yaw_new, s, kappa, vx))

class PathPlotter(Node):
    """
    웨이포인트가 담긴 CSV 파일 경로를 파라미터로 받아,
    해당 웨이포인트를 읽어 MarkerArray로 시각화하여 퍼블리시합니다.
    CSV 파일 형식은 다음과 같습니다.
    x1, y1, yaw1, (speed1)? - 첫 번째 행, speed는 있을 수도 있고 없을 수도 있으며 앞의 3개 값만 추출합니다.

    각 웨이포인트마다 위치 (x, y)와 쿼터니언으로 표현된 자세(yaw)를 가진 Marker를 생성합니다.
    """
    def __init__(self):
        super().__init__('path_plotter')

        # 퍼블리셔를 생성합니다.
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # 웨이포인트 파일을 지정하기 위한 ROS 2 파라미터를 생성합니다.
        self.declare_parameter('waypoint_file', '/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/icra_race_2_interactive.csv')
        waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = np.genfromtxt(waypoint_file, delimiter=',')
        self.waypoints = self.waypoints
        # 첫 번째 점을 배열의 끝에 추가합니다.
        self.waypoints = np.append(self.waypoints, [self.waypoints[0]], axis=0)

        # 웨이포인트에 곡선을 맞춥니다.
        self.waypoints = FitPath(self.waypoints)

        # MarkerArray를 생성합니다.
        self.marker_array = MarkerArray()
        self.marker_array.markers = []

        print("Number of waypoints: ", len(self.waypoints))

        # 각 웨이포인트에 대해 Marker를 생성합니다.
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            if len(self.waypoints) < 50:
                marker.type = marker.ARROW
            else:
                marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0 # 필요 시 np.sin(waypoint[2] / 2)를 사용할 수 있습니다.
            marker.pose.orientation.w = 1.0 # 필요 시 np.cos(waypoint[2] / 2)를 사용할 수 있습니다.
            if len(self.waypoints) < 50:
                marker.scale.x = 1.0
            else:
                marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            if MODE == 2:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif MODE == 1:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            self.marker_array.markers.append(marker)

        # MarkerArray를 퍼블리시합니다.
        self.publisher.publish(self.marker_array)

        # 점 데이터를 CSV 파일로 저장합니다.
        if MODE == 2:
            save_path = '/home/vaithak/Downloads/UPenn/F1Tenth/sim_ws/src/f1tenth_icra_race/waypoints/icra_race_2_interactive_fitted.csv'
            # CSV 파일에 헤더를 추가합니다.
            header = 'x_ref_m,y_ref_m,psi_racetraj_rad,s_racetraj_m,kappa_racetraj_radpm,vx_racetraj_mps'
            np.savetxt(save_path, self.waypoints, delimiter=',', header=header, comments='', fmt='%.4f')

def main(args=None):
    rclpy.init(args=args)
    path_plotter = PathPlotter()
    rclpy.spin(path_plotter)
    path_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

