# 센터라인 추출 실행 순서

1. 손으로 Centerline 추출
```
ros2 launch path_planner checkpoint_recorder_launch.py
```
위 내용에 정규화 -> 가우시안 필터링 자동실행 추가 q누르면 저장


2. fernet 좌표계 형식으로 변환 및 가우시안 필터링으로 속도값 추가
```
python3 src/path_planner/scripts/traj_file_generator.py
```

<div style="text-align: center;">
  <div style="margin-bottom: 10px;">
    <img src="path_planner/data/fitted_waypoints_speed_heatmap.png" width="100%">
    <p>거리 정보 저장</p>
  </div>
</div>

3. 시뮬레이션을 통해 centerline 기준 트랙 폭 저장

```
# 터미널 1
ros2 launch f1tenth simulation_full_system_launch.py
#터미널 2
ros2 run path_planner centerline_logger_node
```
<div style="text-align: center;">
  <div style="margin-bottom: 10px;">
    <img src="path_planner/data/width_log_visualization.png" width="100%">
    <p>거리 정보 저장</p>
  </div>
</div>


4. 위 두 csv 파일 통합
```
python3 src/path_planner/scripts/width_left_right_creator.py
```
<div style="text-align: center;">
  <div style="margin-bottom: 10px;">
    <img src="path_planner/data/final_waypoints_visualization.png" width="100%">
    <p>거리 정보 저장</p>
  </div>
</div>

---

# 시스템 아키텍처 및 데이터 플로우

## 전체 시스템 플로우

```
spliner.py → state_machine.py → controller.py → pure_pursuit.py
(회피경로 생성)  (경로 융합)     (local_waypoints 수신)  (속도 추종)
```

### 1. spliner.py (회피 경로 생성)
- 장애물 감지 및 회피 경로 스플라인 생성
- **회피 경로 전용 속도 스케일링 적용**
  - 외측 추월: 글로벌 속도 × 1.1 (안전)
  - 내측 추월: 글로벌 속도 × 0.8 (위험)
- 토픽: `/planner/avoidance/otwpnts` (OTWpntArray)

### 2. state_machine.py (경로 융합 및 상태 관리)
- 3가지 상태 관리: `GB_TRACK`, `TRAILING`, `OVERTAKE`
- 회피 경로와 글로벌 경로 융합 (`get_spline_wpts()`)
- 토픽: `/state_machine/local_waypoints` (WpntArray)

### 3. controller.py (컨트롤러 노드)
- local_waypoints 구독
- Pure Pursuit 컨트롤러 호출
- 토픽: `/drive` (AckermannDriveStamped)

### 4. pure_pursuit.py (주행 제어)
- L1 거리 계산: `L1 = q_l1 + speed × m_l1`
- 횡방향 오차 기반 속도 조정
- TRAILING 상태 시 PD 제어

---

## 상태별 속도 정의 차이

### GB_TRACK (글로벌 트래킹)
- **속도 소스**: `final_waypoints.csv` 원본 속도
- **경로**: 최적화된 레이싱 라인
- **lookahead**: 기본 파라미터 (m_l1=0.65, q_l1=-0.65)

### OVERTAKE (회피)
- **속도 소스**: CSV 속도 × 스케일링 (0.8 ~ 1.1배)
- **경로**: 스플라인 회피 경로 (0.25m 해상도)
- **특징**:
  - 코너 곡률(kappa) 분석으로 외측/내측 판단
  - 외측 추월 시 1.1배 증속 (안정적)
  - 내측 추월 시 0.8배 감속 (위험)

### TRAILING (추종)
- **속도 소스**: 회피 구간은 스플라인 속도, 나머지는 CSV 속도
- **경로**: 회피 + 글로벌 융합
- **제어**: PD 제어로 목표 간격 1m 유지
  - P gain: 0.5
  - D gain: 0.5

---

## 경로 융합 로직 (state_machine.py)

```python
def get_spline_wpts(self):
    spline_glob = self.glb_wpnts.copy()  # 글로벌 복사

    # 회피 구간 인덱스 범위 계산
    s_start_idx = find_closest_index(wpnts_s_array, avoidance_wpnts[0].s_m)
    s_end_idx = find_closest_index(wpnts_s_array, avoidance_wpnts[-1].s_m)

    # 회피 구간만 교체 (속도 포함)
    for idx in spline_idxs:
        current_s = self.wpnts_s_array[idx]
        closest_avoid_idx = np.argmin(
            np.abs([wpnt.s_m - current_s for wpnt in avoidance_wpnts])
        )
        spline_glob[idx] = avoidance_wpnts[closest_avoid_idx]  # ← 속도 교체!

    return spline_glob
```

**핵심**:
- 글로벌 waypoints (~0.03m 간격)
- 회피 waypoints (~0.25m 간격, 8배 성김)
- 회피 구간만 Nearest Neighbor로 교체

---

## Pure Pursuit Lookahead 파라미터

### L1 Distance 계산
```
L1 = q_l1 + speed × m_l1
L1_clipped = clip(L1, t_clip_min, t_clip_max)
```

### 현재 파라미터 (controller.py)
| 파라미터 | 값 | 단위 | 설명 |
|---------|-----|-----|------|
| m_l1 | 0.65 | 초 | 속도 비례 계수 |
| q_l1 | -0.65 | m | 고정 오프셋 (음수) |
| t_clip_min | 1.0 | m | 최소 lookahead |
| t_clip_max | 7.0 | m | 최대 lookahead |

### 속도별 Lookahead 거리

| 속도 (m/s) | 계산값 (m) | 클리핑 후 (m) |
|-----------|-----------|--------------|
| 1.0 | -0.00 | 1.0 (min) |
| 3.0 | 1.30 | 1.30 |
| 5.0 | 2.60 | 2.60 |
| 8.0 | 4.55 | 4.55 |
| 10.0 | 5.85 | 5.85 |
| 12.0 | 7.15 | 7.0 (max) |

---

## 롤링 문제 원인 분석

### 회피 경로에서 롤링이 심한 이유

1. **속도 급변**
   - 글로벌(CSV) → 회피(0.8배) → 글로벌(CSV)
   - 예: 5m/s → 4m/s → 5m/s

2. **Lookahead 거리 단축**
   - 회피 시 속도 감소 → L1 거리 감소
   - 5m/s: L1=2.6m → 4m/s: L1=1.95m (25% 감소)
   - 짧은 lookahead → 급격한 조향

3. **회피 경로의 낮은 해상도**
   - 글로벌: ~0.03m 간격
   - 회피: ~0.25m 간격 (8배 성김)
   - 성긴 waypoints → 조향 떨림

### 해결 방안

#### 방안 1: Lookahead 파라미터 증가
```python
# controller.py에서 수정
self.declare_parameter('m_l1', 0.8)      # 0.65 → 0.8
self.declare_parameter('q_l1', 0.5)      # -0.65 → 0.5
self.declare_parameter('t_clip_min', 2.5) # 1.0 → 2.5
self.declare_parameter('t_clip_max', 12.0) # 7.0 → 12.0
```

#### 방안 2: 회피 경로 전용 Lookahead 추가
```python
# 상태별 다른 lookahead 파라미터 사용
if self.state == StateType.OVERTAKE:
    L1_distance = self.q_l1_overtake + self.speed_now * self.m_l1_overtake
else:
    L1_distance = self.q_l1 + self.speed_now * self.m_l1
```

#### 방안 3: 속도 스케일링 완화
```python
# spliner.py에서 수정
# 현재: 1.1배 / 0.8배 (30% 차이)
# 권장: 1.05배 / 0.9배 (15% 차이)
vi = self.waypoints_v[gb_wpnt_i] * 1.05 if outside == more_space \
     else self.waypoints_v[gb_wpnt_i] * 0.9
```