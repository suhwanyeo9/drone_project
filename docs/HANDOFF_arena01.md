# 인수인계 — arena01 환경 + 드론-로봇 통합 파이프라인

작성일: 2026-08-09 / 사용자: godns@godns-DT281 (Ubuntu, ROS 2 Humble)
**이 문서가 최신본. `HANDOFF_slam_room.md`(8/3자)는 폐기 — 지도가 yellow_test → arena01로 교체되면서 파일명·수치·실행절차가 전부 바뀜.**

---

## 1. 목적과 현재 도달점

실물 MentorPi를 돌리는 실제 방을 Gazebo에 재현하고, **드론 카메라가 목표물을 찾아 좌표를 넘기면 지상 로봇이 순차 방문하는** 전체 파이프라인을 구축.

**시뮬레이션 범위에서는 완료.** 드론 탐사 → 검출 → 지면 좌표 변환 → 타겟 확정 → Nav2 순차 방문 → 홈 복귀까지 전 구간이 재현성 있게 동작.

최종 실행 결과 (2026-08-09 재부팅 후 재현 확인):
```
확정 3/3 | #1(+2.25,+2.05), #2(+1.85,-2.02), #3(-0.74,+0.21)
■■ 미션 완료 — 3개 중 방문 처리 3개 + 홈 복귀, 소요 97.3초
```
좌표 정확도 정답 대비 **2cm 이내**.

---

## 2. 실행 방법 — 이것만 보면 됨

터미널 3개.

### T1 — 무대 (Gazebo + 로봇 + 드론 + 검출기 + 좌표계산 + 미션매니저 + Nav2)
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch ~/drone_project/launch/arena01_phase4_base.launch.py approach_offset:=0.5
```

### T2 — RViz
```bash
rviz2 -d ~/drone_project/rviz/arena01.rviz --ros-args -p use_sim_time:=true
```

### T3 — 시나리오
```bash
cd ~/drone_project
bash scripts/run_scenario.sh -2 2 -2 2 -2.5 1.8 0.8 1.5
```
인자: `x_min x_max y_min y_max home_x home_y speed pause`

### 기동 확인
```bash
ros2 topic list | grep -E "model_states|obstacle|detection"
```
`/gazebo/model_states`, `/detection/pixel`, `/obstacle/ground_points` 3개가 나와야 정상.

> **주의**: `map_server`나 `nav2_bringup`을 따로 띄우면 안 됨. 무대 런치에 다 들어 있음.

---

## 3. 핵심 수치 (arena01)

| 항목 | 값 |
|---|---|
| 원본 SLAM 지도 | `maps_src/arena01.pgm` 113×195 px, res 0.05, origin [-3.75, -5.87, 0] |
| 실측 방 크기 | 3.05 × 3.00 m (실측 3×3m와 일치) |
| 확대 후 방 내부 | **6.10 × 6.00 m** (넓이 4배), 벽 포함 6.30 × 6.20 m |
| 좌표 원점 | 방 중심 = Gazebo (0,0) |
| Nav2 지도 | 126 × 124 px @ 0.05, origin **[-3.150, -3.100, 0.0]** |
| 로봇 스폰 | (-2.0, 0.0), yaw 0 |
| 권장 홈 좌표 | **(-2.5, 1.8)** |
| 드론 카메라 | pose `0 0 8 0 1.5708 0`, HFOV 1.047, 640×480, fx 554.38, cx 320.5, cy 240.5 |
| 카메라 토픽/프레임 | `/drone/image_raw` / `drone_camera_optical` |
| 지면 축척 | 0.014430 m/px (고도 8m 기준) |

### 목표 기둥 (빨강, 반경 0.3m, 높이 1.0m)
```
target_spartina    (-0.75,  0.20)
target_spartina_2  ( 2.25,  2.05)
target_spartina_3  ( 1.85, -2.05)
```

### 방 안 장애물 4개 (확대 후, 중심 x,y / 가로×세로)
```
A (-1.16,  1.62)  0.60 × 0.60
B ( 1.18,  1.54)  0.80 × 0.70
C (-1.30, -0.80)  0.70 × 0.80
D ( 1.20, -1.46)  0.50 × 0.70
```

### 좌표계
```
map ≡ odom ≡ Gazebo 월드 (static_tf_map_to_odom identity)
map → odom → base_footprint → base_link → (base_scan, camera_link, wheel_*)
map → drone_camera_optical  (drone_tf_broadcaster.py가 동적 발행)
```

### 지면 투영 변환식
```
X_map = -(v - cy) * s
Y_map = -(u - cx) * s
s = (drone_z - plane_z) / fx,  plane_z = target_height * HEIGHT_RATIO(2/3)
```
`target_height:=1.0`을 줘야 시차 보정이 적용됨. 안 주면 기본 0.0 = 기존 동작(오차 0.07~0.27m).

---

## 4. 절대 건드리지 말 것

### `config/nav2_explore.yaml`
```yaml
inflation_radius: 0.350      # global/local 둘 다
cost_scaling_factor: 3.0
xy_goal_tolerance: 0.25
```
**둘 다 실험해서 악화됨을 확인했음.**
- `inflation_radius` 0.250 → 미션 516초에 2/3만 방문 (팽창을 줄이면 전역경로가 장애물에 붙어 DWB가 못 따라감)
- `xy_goal_tolerance` 0.15 → 도착 판정 실패로 `Failed to make progress` 14회 반복 정체

### `approach_offset`
런치 인자로 **0.5**. 물리 하한 0.45m(기둥 반경 0.30 + waffle 반폭 0.14)이라 더 낮추면 물리적으로 못 감.

---

## 5. 파일 구조

```
~/drone_project/
├── make_arena01.py                        ← 지도 → world + Nav2 지도 생성기
├── maps_src/arena01.pgm, .yaml            ← 원본 SLAM 지도
├── worlds/arena01.world                   ← 생성된 3D 환경
├── maps/arena01_map.pgm, .yaml            ← 생성된 Nav2 지도
├── rviz/arena01.rviz                      ← costmap/path 포함 저장본
├── config/nav2_explore.yaml               ← Nav2 설정
├── launch/
│   ├── arena01_phase4_base.launch.py      ← 정본 무대
│   ├── arena01_multi.launch.py            ← 무대가 include하는 하위 런치
│   └── arena01_nav2.launch.py             ← 기본 주행만 (드론 파이프라인 없음)
└── scripts/
    ├── run_scenario.sh                    ← 시나리오 실행
    ├── drone_survey_once_node.py          ← 드론 1회 탐사
    ├── drone_tf_broadcaster.py            ← map→drone_camera_optical 동적 TF
    ├── spartina_detector_multi_node.py    ← 빨강 HSV 검출 → /detection/pixel
    ├── obstacle_locator_multi_node.py     ← 픽셀 → map 좌표
    ├── obstacle_locator_node.py           ← 위 노드가 import하는 base 모듈
    ├── target_manager_node.py             ← 타겟 확정/중복 제거
    ├── mission_manager_node.py            ← 접근점 계산 + Nav2 goal 전송
    └── (그 외: click_pixel_node, goal_relay_node, target_follower_node,
          validate_projection, pixel_to_ground, spartina_detector_node,
          drone_survey_node, obstacle_locator_multi_node)
```

**팀 원본(`spartina_*`)은 그대로 두고 `arena01_*` 복사본을 쓰는 방식.** 원본을 건드리지 않음.

### 관련 커밋 (브랜치 `phase4-multi-target`)
| 커밋 | 내용 |
|---|---|
| `4ff9a4f` | arena01 기반 시뮬 환경 생성 |
| `e6d7ab5` | obstacle_locator_node에 `plane_z` / `target_height` 추가 (시차 보정) |
| `8f1fc9d` | mission_manager에 costmap 기반 접근점 탐색(`_pick_approach`) 추가 |
| `47c6f51` | 미탐색(-1) 셀 허용 패치 |

백업본: `*.bak` (obstacle_locator_node, obstacle_locator_multi_node, mission_manager_node, run_scenario.sh)

---

## 6. 겪은 함정 — 반복하지 말 것

| 함정 | 증상 | 원인/해결 |
|---|---|---|
| `libgazebo_ros_state.so` 누락 | 드론이 로그만 찍고 안 움직임, "카메라 TF 조회 실패" 반복 | world 레벨 플러그인(namespace `/gazebo`, update_rate 50) 필수. 없으면 `/gazebo/model_states`, `/gazebo/set_entity_state`가 아예 안 생김 |
| 홈 좌표가 항상 (-3,-3) | 로봇이 벽 코너로 감 | `run_scenario.sh`가 `/mission/reset`을 param set보다 **먼저** 보내서 옛 값을 읽었던 것. 파라미터 블록을 리셋 앞으로 이동해 해결 |
| `_pick_approach` 탐색 실패 | "직선 방식으로 대체" 남발 | phase4는 static layer 없는 **롤링 costmap**이라 미탐색 셀이 -1. `c < 0`을 탈락시키면 안 봤을 뿐인 곳을 전부 막힘 처리 |
| RViz costmap 400×400 | 환경이 바뀐 줄 착각 | phase4는 map_server가 없어 항상 400×400(20m 롤링). **환경 구분 지표가 아님** |
| 홈 (-2.5, 0.0) | 복귀 경로가 모서리에 끼임 | 장애물 C(y -1.20~-0.40)와 같은 y 대역. `_go_home`엔 우회 로직 없음. (-2.5, 1.8) 사용 |
| 잘못된 런치 | slam_room 환경이 뜸 | `arena01_phase4_base.launch.py`가 정본. `slam_room_nav2.launch.py`는 구버전 |

---

## 7. 성능 한계 (개선 시도 후 확정)

| 타겟 | 최근접 거리 | 비고 |
|---|---|---|
| #1 (2.25, 2.05) | 0.43 m | 물리 한계 도달 |
| #2 (1.85, -2.05) | 0.74 m | 장애물 D와 기둥 사이 틈 0.10m라 북쪽 직행 불가 |
| #3 (-0.75, 0.20) | 0.45 m | 물리 한계 도달 |
| 홈 복귀 | 0.24 m | 허용오차 이내 |

#2의 0.74m = `approach_offset 0.5 + xy_goal_tolerance 0.25` 경계값. 더 줄이려면 tolerance를 낮춰야 하는데 그건 정체를 유발하므로 **여기서 마무리하는 것이 합리적**.

---

## 8. 남은 일 — 전부 "실물로 나가는" 방향

시뮬레이션 안에서 할 일은 끝났음. 남은 3개는 모두 하드웨어나 실데이터가 필요.

### ① 실물 오도메트리 캘리브레이션 (미착수)
SLAM 지도의 전단(스큐)이 yellow_test 6.22° → arena01 2.4°로 줄었지만, **원인인 바퀴 지름 / 트레드 폭 캘리브레이션은 손도 안 댐.** 새로 매핑하면서 우연히 나아진 것.
지금까지 한 건 시뮬 쪽에서 스큐를 지운 것이지 실물 오차를 고친 게 아님. 실물로 옮기면 그대로 나타남.

**측정 방법**: 직선 2m 주행 후 실제 이동거리 대비 `/odom` 값 → 바퀴 지름 보정. 제자리 360° 회전 후 각도 오차 → 트레드 폭 보정.

### ② 실제 갯끈풀 검출기 (미착수)
현재 `spartina_detector_multi_node.py`는 **빨강 HSV 색상 검출**(H 0-10 / 160-180, S≥100, V≥60). 시뮬 기둥이 빨간색이라 동작하는 것.
실제 갯끈풀은 초록색 식물이고 갯벌·다른 식물과 색으로 구분 안 됨. **이 노드를 통째로 교체해야 함.**

다행히 인터페이스는 잘 잡혀 있음 — 새 검출기가 `/detection/pixel` (PointStamped, u/v)만 발행하면 좌표 변환·미션 관리는 그대로 재사용 가능.

### ③ 실물 MentorPi 전체 검증 (미착수)
지금까지 전부 Gazebo 안에서만 돌림. 원래 목적이 "시뮬과 실물을 일치시켜 결과를 옮기는 것"인데 그 마지막 단계가 남음.

---

## 9. 참고 — 환경 재생성이 필요할 때

```bash
python3 ~/drone_project/make_arena01.py
```
`maps_src/arena01.pgm`에서 `worlds/arena01.world` + `maps/arena01_map.pgm/.yaml`을 다시 만듦.
스큐·회전은 재현하지 않고 **완전한 직사각형**으로 생성. 목표 기둥은 Nav2 지도에 미포함(라이다가 실시간으로 잡게).
