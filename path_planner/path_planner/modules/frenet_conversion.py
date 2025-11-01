from typing import Union
import numpy as np
from scipy.interpolate import CubicSpline

class FrenetConverter:
    def __init__(self, waypoints_x: np.array, waypoints_y: np.array, waypoints_psi: np.array):
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.waypoints_psi = waypoints_psi
        self.waypoints_s = None
        self.spline_x = None
        self.spline_y = None
        self.raceline_length = None
        self.waypoints_distance_m = 0.1  # [m] 단위
        self.iter_max = 3
        self.closest_index = None

        self.build_raceline()

    def build_raceline(self):
        self.waypoints_s = [0.0]
        prev_wpnt_x =  self.waypoints_x[0]
        prev_wpnt_y =  self.waypoints_y[0]
        for wpnt_x, wpnt_y in zip(self.waypoints_x[1:], self.waypoints_y[1:]):
            dist = np.linalg.norm([wpnt_x - prev_wpnt_x, wpnt_y - prev_wpnt_y])
            prev_wpnt_x = wpnt_x
            prev_wpnt_y = wpnt_y
            self.waypoints_s.append(self.waypoints_s[-1] + dist)        
        self.waypoints_s = np.array(self.waypoints_s)
        self.spline_x = CubicSpline(self.waypoints_s, self.waypoints_x)
        self.spline_y = CubicSpline(self.waypoints_s, self.waypoints_y)
        self.raceline_length = self.waypoints_s[-1]

    def get_frenet(self, x, y, s=None) -> np.array:
        # 주어진 (x, y) 점에 대한 프레네 좌표를 계산합니다.
        self.closest_index = self.get_closest_index(x, y)  # 다른 기능에서 사용할 인덱스를 갱신합니다.
        if s is None:
            s = self.get_approx_s(x, y)
            s, d = self.get_frenet_coord(x, y, s)
        else:
            s, d = self.get_frenet_coord(x, y, s)

        return np.array([s, d])

    def get_approx_s(self, x, y) -> float:
        """
        가장 가까운 웨이포인트를 찾아 주어진 점의 s 좌표를 추정합니다.
        """
        # 가장 가까운 웨이포인트를 찾습니다.
        closest_index = self.closest_index
        return self.waypoints_s[closest_index]
    
    def get_frenet_velocities(self, vx, vy, theta) -> np.array:
        """
        주어진 직교 속도로부터 프레네 속도 성분을 계산합니다.
        
        Args:
            vx (float): x 방향 속도
            vy (float): y 방향 속도
            theta (float): 차량의 요 각도
            
        Returns:
            np.array: [s_dot, d_dot] 형태의 프레네 속도
        """
        if self.closest_index is None:
            raise ValueError("FRENET CONVERTER: closest index is None, call get_closest_index first.")
        delta_psi = theta - self.waypoints_psi[self.closest_index]
        s_dot = vx * np.cos(delta_psi) - vy * np.sin(delta_psi)
        d_dot = vx * np.sin(delta_psi) + vy * np.cos(delta_psi)
        
        return np.array([s_dot, d_dot])

    def get_closest_index(self, x, y) -> int:
        """
        주어진 점에 가장 가까운 웨이포인트의 인덱스를 찾습니다.
        
        Args:
            x (float): 점의 x 좌표
            y (float): 점의 y 좌표
            
        Returns:
            int: 가장 가까운 웨이포인트 인덱스
        """
        # 배열 브로드캐스팅을 이용해 거리 행렬을 계산합니다.
        lenx = len(x)
        dist_x = x - np.tile(self.waypoints_x, (lenx, 1)).T
        dist_y = y - np.tile(self.waypoints_y, (lenx, 1)).T
        self.closest_index = np.argmin(np.linalg.norm([dist_x.T, dist_y.T], axis=0), axis=1)
        return self.closest_index


    def get_frenet_coord(self, x, y, s, eps_m=0.01) -> float:
        """
        트랙에 수직으로 투영하여 주어진 점의 s 좌표를 세밀하게 보정합니다.
        
        Args:
            x (float): 점의 x 좌표
            y (float): 점의 y 좌표
            s (float): 추정된 s 좌표
            eps_m (float): 투영 오차 허용치(기본값 0.01)
        
        Returns:
            트랙 상에서의 보정된 s 좌표
        """
        # 추정한 s에서 트랙에 수직으로 위치하는지 확인합니다.

        _, projection, d = self.check_perpendicular(x, y, s, eps_m)
        for i in range(self.iter_max):
            cand_s = (s + projection)%self.raceline_length
            _, cand_projection, cand_d = self.check_perpendicular(x, y, cand_s, eps_m)
            #print(f"candidate projection: {cand_projection}; projection: {projection}; d: {d} cand_d: {cand_d}")
            cand_projection = np.clip(cand_projection, -self.waypoints_distance_m/(2*self.iter_max), self.waypoints_distance_m/(2*self.iter_max))
            updated_idxs = np.abs(cand_projection) <= np.abs(projection)
            d[updated_idxs] = cand_d[updated_idxs]
            s[updated_idxs] = cand_s[updated_idxs]
            projection[updated_idxs] = cand_projection[updated_idxs]

        return s, d

    def check_perpendicular(self, x, y, s, eps_m=0.01) -> Union[bool, float]:
        # 트랙에 평행한 단위 벡터를 계산합니다.
        dx_ds, dy_ds = self.get_derivative(s)
        tangent = np.array([dx_ds, dy_ds])
        if np.any(np.isnan(s)):
            raise ValueError("BUB FRENET CONVERTER: S is nan")
        tangent /= np.linalg.norm(tangent, axis=0)

        # 트랙에서 해당 점까지의 벡터를 구합니다.
        x_vec = x - self.spline_x(s)
        y_vec = y - self.spline_y(s)
        point_to_track = np.array([x_vec, y_vec])
        
        # 두 벡터가 수직인지 확인합니다.
        # point_to_track 벡터를 접선에 사영합니다.
        proj = np.einsum('ij,ij->j', tangent, point_to_track)
        perps = np.array([-tangent[1, :], tangent[0, :]])
        d = np.einsum('ij,ij->j', perps, point_to_track)

        # TODO 계산 비용 때문에 비활성화되어 있습니다.
        # 스케일 불변성을 위해 eps_m * point_to_track_norm 조건을 사용하면
        # 사실상 cos(angle) <= eps_m 조건으로 해석할 수 있습니다.

        # point_to_track_norm = np.linalg.norm(point_to_track, axis=0)
        # check_perpendicular = np.abs(proj) <= eps_m * point_to_track_norm
        check_perpendicular = None

        return check_perpendicular, proj, d
    
    def get_derivative(self, s) -> np.array:
        """
        주어진 s에서의 좌표 미분(dx/ds, dy/ds)을 반환합니다.
        
        Args:
            s: 스플라인을 평가하기 위한 파라미터

        Returns:
            der: [dx/ds, dy/ds]
        """
        s = s%self.raceline_length

        der = [self.spline_x(s, 1), self.spline_y(s, 1)]
        
        return der
    

    def get_cartesian(self, s: float, d: float) -> np.array:
        """
        프레네 좌표를 직교 좌표로 변환합니다.
        
        Args:
            s (float): 종방향 좌표
            d (float): 횡방향 좌표
            
        Returns:
            np.array: [x, y] 형태의 직교 좌표
        """
        x = self.spline_x(s)
        y = self.spline_y(s)
        psi = self.get_derivative(s)
        psi = np.arctan2(psi[1], psi[0])
        x += d * np.cos(psi + np.pi / 2)
        y += d * np.sin(psi + np.pi / 2)
        
        return np.array([x, y])
    
