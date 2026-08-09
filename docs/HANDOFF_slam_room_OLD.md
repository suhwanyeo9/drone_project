# 인수인계 — 실물 SLAM 지도 기반 Gazebo 환경 구축

작성일: 2026-08-03 / 사용자: godns@godns-DT281 (Ubuntu, ROS 2 Humble)

---

## 1. 목적

실물 MentorPi 로봇을 돌리는 **실제 방**과 **Gazebo 시뮬레이션 환경**을 일치시킨다.
기존에는 직육면체 3개 + 빨간 기둥으로 임의 구성한 환경이라 시뮬 결과가 실물로 전이되지 않았음.

**중요 제약**: 기존 spartina 파이프라인(로봇 스폰, 드론 카메라, 좌표 계산 노드, TF)은 **절대 건드리지 않는다.**
바꾸는 것은 **Gazebo world 파일 경로 한 줄뿐**이다.

---

## 2. 현재 상태

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | SLAM 지도에서 직사각형 방 추출 | ✅ 완료 |
| 2 | 스큐 6.22° 보정 | ✅ 완료 |
| 3 | 넓이 4배 확대 (2.85×2.95m → 5.70×5.90m) | ✅ 완료 |
| 4 | 3D world 생성 (벽 4장 + 장애물 47개) | ✅ 완료 |
| 5 | 드론 카메라 모델 이식 | ✅ 완료 |
| 6 | Nav2용 2D 지도 역생성 | ✅ 완료 |
| 7 | Gazebo 실행 검증 | ✅ 완료 (RTF 1.00, 55 models) |
| 8 | RViz에서 라이다↔지도 겹침 검증 | ✅ **완료 — 정확히 겹침** |
| 9 | Nav2 기동 + costmap 확인 | ✅ 완료 (통로 여유 충분) |
| 10 | 2D Pose Estimate → 2D Goal Pose 주행 | ⏳ **여기서 중단됨 (PC 멈춤)** |
| 11 | 드론 카메라 → YOLO → goal 연동 | ⏳ 미착수 |

---

## 3. 생성된 파일

```
~/drone_project/
├── worlds/slam_room.world              ← 새 Gazebo 환경 (33,570 bytes)
├── maps/slam_room_map.pgm              ← Nav2 지도 이미지 (17,435 bytes, 130×134 px)
├── maps/slam_room_map.yaml             ← 지도 메타데이터 (136 bytes)
├── launch/slam_room_nav2.launch.py     ← spartina_nav2.launch.py 복사본, world 경로만 교체
└── rviz/slam_room.rviz                 ← RViz 설정 (Costmap/Path 항목은 미저장)
```

### 원본 SLAM 파일
- `yellow_test.pgm` (112×198 px), `yellow_test.yaml` (resolution 0.05, origin [-2.39, -4.61, 0])
- 지도 전체는 5.6×9.9m이지만 **그중 닫힌 직사각형 방만** 사용 (원본 픽셀 row 48–112, col 46–108)

---

## 4. 핵심 수치 — 반드시 유지할 값

| 항목 | 값 |
|---|---|
| 실제 방 크기 | 2.85 × 2.95 m (사용자 실측 3×3m와 일치) |
| 확대 후 방 내부 | **5.70 × 5.90 m** (넓이 4배, 길이 2배) |
| 좌표 원점 | **방 중심 = Gazebo (0,0)** |
| 지도 크기 | 130 × 134 px @ 0.05 m/cell |
| 지도 origin | **`[-3.250, -3.350, 0.0]`** |
| 로봇 스폰 | x = **-2.0**, y = **0.0**, yaw = **0** (기존 launch 기본값 그대로, 여유 0.84m) |
| 드론 카메라 | pose `0 0 8 0 1.5708 0`, HFOV 1.047, 640×480 |
| 카메라 토픽 | **`/drone/image_raw`**, `/drone/camera_info` (`/camera/*` 아님) |
| 카메라 프레임 | `drone_camera_optical` |
| 목표 기둥 | `target_spartina` (-0.75, -0.15), `target_spartina_2` (2.25, 2.05), `target_spartina_3` (1.85, -2.05), 반지름 0.3m 높이 1.0m |

### 좌표계 구조 (검증 완료)
```
map ≡ odom ≡ Gazebo 월드 좌표계 (전부 방 중심이 원점)
```
- `spartina_nav2.launch.py`의 `static_tf_map_to_odom`이 identity(0,0,0)
- TurtleBot3 diff_drive 플러그인이 odom을 **Gazebo 원점 기준**으로 발행
- 검증: 스폰 후 `/odom` 값이 `x: -1.9998, y: 0.0000` → 스폰 좌표 그대로 찍힘
- 따라서 지도 origin은 스폰 오프셋을 빼지 않은 `[-3.25, -3.35]`가 맞음

### 카메라 화각 검증
8m 높이에서 지면 촬영 범위: 월드 Y축 9.24m, 월드 X축 6.93m
→ 방(5.90 × 5.70m)이 전부 화면에 들어옴. **카메라 높이 조정 불필요.**

---

## 5. 실행 절차

터미널 4개 필요. **순서대로, 앞 터미널을 끄지 말 것.**

### T1 — Gazebo + 로봇 + 드론 카메라
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch ~/drone_project/launch/slam_room_nav2.launch.py
```
로봇은 5초 뒤 스폰됨 (`TimerAction(period=5.0)`).

### T2 — 지도 서버
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=$HOME/drone_project/maps/slam_room_map.yaml \
  -p use_sim_time:=true
```
별도 터미널에서 활성화 (lifecycle 노드라 필수):
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### T3 — RViz
```bash
rviz2 -d ~/drone_project/rviz/slam_room.rviz --ros-args -p use_sim_time:=true
```

### T4 — Nav2
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
정상 기동 확인 로그:
```
StaticLayer: Resizing costmap to 130 X 134 at 0.050000 m/pix
Managed nodes are active
```

---

## 6. RViz 설정 — 함정 주의

| 표시 항목 | Topic | **반드시 바꿔야 할 것** |
|---|---|---|
| `Map` | `/map` | Durability Policy → **`Transient Local`** |
| `Costmap` (Map 타입) | `/global_costmap/costmap` | Durability → **`Transient Local`**, Color Scheme → `costmap`, Alpha 0.5 |
| `LaserScan` | `/scan` | Reliability Policy → **`Best Effort`**, Size 0.05, FlatColor 빨강 |
| `Path` | `/plan` | Color 초록 |
| `RobotModel` | `/robot_description` | — |

- Fixed Frame = **`map`**
- `Reliability`/`Durability`는 `Topic` 항목을 **한 번 더 펼쳐야** 보임
- 이 설정 안 하면 화면이 텅 빈 것처럼 보임 (가장 흔한 삽질 지점)

---

## 7. 다음 할 일

### 즉시 (10단계)
1. RViz 상단 **`2D Pose Estimate`** → 지도 왼쪽 중간(로봇 위치)을 클릭 드래그
   - T4에 `Received initial pose` 로그 확인
2. RViz 상단 **`2D Goal Pose`** → 회색 빈 공간(추천: 오른쪽 위 모서리) 클릭 드래그
3. 초록 경로선 생성 + Gazebo에서 로봇 이동 확인

**실패 시 로그별 대응**
| 로그 | 원인 | 대응 |
|---|---|---|
| `Failed to create a plan` | costmap이 통로를 막음 | `inflation_radius` 축소 (기본 0.55 → 0.35) |
| `Goal was rejected` | 목표가 금지구역 | 회색 영역에 다시 찍기 |
| `Timed out waiting for transform` | use_sim_time 누락 | 모든 노드에 `use_sim_time:=true` |

### 이후 (11단계)
- `obstacle_locator_node.py`를 `world_frame:=map`으로 실행
- 드론 카메라 → YOLO 검출 → 좌표 변환 → Nav2 goal 연동
- `spartina_phase4.launch.py` / `mission_manager_node.py` 연계

---

## 8. 저장 안 된 것 / 주의사항

### 저장 안 됨
1. **RViz의 `Costmap`, `Path` 항목** — 저장 후에 추가해서 `.rviz` 파일에 없음. 재실행 시 `Add`로 다시 넣어야 함 (6번 표 참고)
2. **git 커밋** — `~/drone_project`에 새 파일들이 아직 커밋 안 됨
   ```bash
   cd ~/drone_project
   git add worlds/slam_room.world maps/slam_room_map.* launch/slam_room_nav2.launch.py rviz/
   git commit -m "실제 방 기반 시뮬 환경 추가"
   ```

### 재현 스크립트 (필요 시)
- `make_world.py` — SLAM pgm → world 재생성 (파라미터: `AREA_SCALE`, `OBSTACLE_SCALE`, `DESKEW`, `WALL_H`, `OBST_H`)
- `make_map.py` / `gen_map_local.py` — world → Nav2 지도 재생성
- `gen_map_local.py`는 numpy만 필요 (scipy/PIL 불필요), 로컬 실행용

### 겪은 함정
- 브라우저가 동명 파일을 `slam_room(1).world` 식으로 저장함 → `grep -c "libgazebo_ros_camera"` 로 반드시 검증 (1이 나와야 함)
- `map_server`는 lifecycle 노드라 `configure` + `activate` 둘 다 필요
- 기존 world의 `spartina_world.world`는 그대로 두고 있음 (롤백용)

---

## 9. 미해결 관찰 사항

**SLAM 지도가 6.22° 전단(shear) 되어 있었음.** 좌우 벽은 수직인데 상하 벽만 같은 방향으로 기울어짐 = 회전이 아니라 스큐.
실제 방이 직사각형이므로 이건 **실물 로봇의 오도메트리 오차**로 추정됨. 보통 바퀴 지름 또는 트레드 폭 캘리브레이션 문제.
시뮬 world에는 보정을 적용했지만, **실물 SLAM 정확도를 올리려면 별도로 캘리브레이션이 필요함.**
