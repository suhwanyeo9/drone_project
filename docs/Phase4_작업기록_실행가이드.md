# Phase 4 작업 기록 & 실행 가이드

> 담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3)
> 작성일: 2026-07-26 · 브랜치: `phase4-multi-target` (suhwanyeo9/drone_project)
> 목표: 드론 자율 탐사 → 다중 목표 자동 탐지 → 로봇 순차 방문 (전 과정 무인)

---

## 1. 오늘 완료한 것 요약

| 단계 | 내용 | 실측 결과 |
| --- | --- | --- |
| Step 0 | 선배(정우열) Phase 1~3 재현 검증 | 게이트 5개 전부 통과 |
| Step 1 | 다중 목표 월드 + 탐지·투영 다중화 | 목표 3개 각각 발행, 투영 폐기 0건 |
| Step 2 | 클러스터링 중복 제거 (target_manager) | 수신 1,866건 → 확정 3개, 유령 후보 0 |
| Step 3 | Nav2 액션 기반 순차 방문 (mission_manager) | 3/3 방문, 소요 31.0초 |
| Step 4 | 통합 무인 데모 (드론 탐사 + 로봇 주행 병행) | 소요 32.0초, 사람 클릭 0회 |

### 커밋 이력 (phase4-multi-target)

1. `Phase4: 다중 목표 월드 추가 (target 3개: (2,0), (-2,2), (0,-2))`
2. `Phase4 Step1: 다중 목표 탐지·투영 (컨투어별 발행 + 즉시투영 + 재시도큐, 실패 0건 실측)`
3. `Phase4 Step2: target_manager 클러스터링 (이동 중 1866건 수신 → 확정 3개 안정, 유령 후보 0)`
4. `Phase4 Step3: mission_manager (NavigateToPose 액션, greedy 순회, 3/3 방문 31초 실측)`
5. `Phase4: mission_manager v3 — 홈 복귀 + 복귀 좌표 파라미터 지정 (use_home_pose/home_pose)`
6. `Phase4: 완전 시나리오 — 드론 1회탐사+복귀+정지(DONE), 로봇은 탐사완료 후 출발, 런치 인자로 범위/복귀좌표 지정`
7. `Phase4: 무대/공연 분리 — base 런치 + run_scenario.sh (리셋·재실행, 런타임 좌표 갱신, 정수→실수 자동변환)`

---

## 2. 새로 만든 파일 (기존 파일은 전부 미수정)

| 파일 | 역할 | 핵심 설계 |
| --- | --- | --- |
| `worlds/spartina_world_multi.world` | 목표 3개 월드: (2,0), (-2,2), (0,-2) | 목표 간 최소 거리 2.83m — 클러스터 혼동 불가 배치 |
| `launch/spartina_multi.launch.py` | multi 월드용 런치 (moving 런치 복사, world 경로만 변경) | |
| `scripts/spartina_detector_multi_node.py` | 색 탐지기 다중화 | 최대면적 1개 → min_area 이상 전 컨투어 각각 발행. 가장자리 30px 필터(이상치 차단). 발행 제한은 프레임 단위 |
| `scripts/obstacle_locator_multi_node.py` | 좌표 계산 다중화 | 선배 locator 를 import·상속. 탐지 즉시 투영(덮어쓰기 구조 제거) + 비블로킹 재시도 큐 + 초기 유령 목표 제거. 출력: `/obstacle/ground_points` |
| `scripts/target_manager_node.py` (v3) | 클러스터링·확정 관리 | merge_radius 0.5m 병합(누적 평균), 관측 3회 이상 확정. 출력: `/targets/confirmed`. v2: 드론 탐사 시작 후에만 등록(require_survey). v3: /targets/reset 리셋 |
| `scripts/mission_manager_node.py` (v5) 🚗 | 다중 목표 순차 방문 + 홈 복귀 | NavigateToPose 액션, greedy 순서, 접근 오프셋 0.6m, 재시도 1회 후 skip. 홈 복귀(RETURN) — 좌표는 자동 또는 지정. v4: 드론 탐사 DONE 대기 후 출발. v5: /mission/reset 리셋 + 리셋 시 복귀 좌표 재로딩 |
| `scripts/drone_survey_once_node.py` 🚁 | 1회 탐사 + 복귀 + 정지 | 지정 범위 지그재그 한 바퀴 → 이륙 지점 자동 복귀 → 정지. 상태(MOVING/HOLD/RETURN/DONE)를 /drone/survey_state 로 발행 — DONE 이 로봇 출발 신호 |
| `launch/spartina_phase4_base.launch.py` 🎭 | 무대 런치 | Gazebo+Nav2+파이프라인+매니저만 기동 (드론 탐사 제외). 한 번 켜두고 공연을 반복 |
| `scripts/run_scenario.sh` 🎬 | 공연 스크립트 | 무대 위에서 시나리오 실행/재실행: 리셋 → 복귀 좌표 런타임 갱신 → 드론 원점 복귀 → 탐사 실행. 인자: x_min x_max y_min y_max home_x home_y speed hold_sec (정수 입력 자동 실수 변환) |

### 데이터 흐름

```
드론 카메라 /drone/image_raw
  → [spartina_detector_multi] /detection/pixel (PointStamped, 영상 stamp 유지 ★규약)
  → [obstacle_locator_multi]  /obstacle/ground_points (낱개 map 좌표)
  → [target_manager]          /targets/confirmed (확정 타겟 PoseArray)
  → [mission_manager]         Nav2 NavigateToPose 액션 → 로봇 순차 방문
```

---

## 3. 오늘 발견·해결한 문제 (보고서 재료)

**① locator 단일 목표 덮어쓰기 (구조적 한계 실측)**
원본 locator 는 탐지 픽셀을 상태 변수 하나에 저장 → 탐지 3건이 연달아 오면
마지막 1건만 투영됨 (실측: target 1 만 확정, 2·3 유실).
→ 해결: 탐지 콜백에서 즉시 투영하는 구조로 변경 (저장 단계 자체를 제거).

**② 콜백 내 블로킹 대기로 인한 실행자 기아**
탐지 콜백 안에서 TF lookup_timeout(100ms) 대기 → 초당 15건 탐지에서 대기
시간이 처리량 초과 → TF 콜백이 실행될 틈을 잃어 버퍼가 계속 빔 → 전부 실패
(실측: 투영 0건 / 실패 400+건).
→ 1차: `lookup_timeout:=0.0` → 성공률 75% (카메라 10Hz vs TF 50Hz 시각차로
   몇 ms 앞서는 순간 잔존 실패 25%).
→ 최종: 비블로킹 재시도 큐 — 실패 건을 쟁여뒀다 0.1s 후 재시도, 0.5s 초과 폐기.
   실측: **폐기 0건**.

**③ 초기 유령 목표**
원본 locator 가 시작 직후 화면 중앙(320,240)을 투영 → 드론 바로 아래 지점이
가짜 목표로 등록됨 (실측: map(+0.01,+0.01)). 다중 목표 리스트에 들어가면
로봇이 엉뚱한 곳 방문. → 다중판에서는 실제 탐지 전까지 아무것도 안 하도록 제거.

**④ 화면 가장자리 탐지 이상치**
목표가 화면 밖으로 반쯤 잘릴 때 보이는 부분만의 무게중심이 계산되어 좌표가
밀림 (Step 0 실측: 정상 1.9~2.17 분포에서 1.69 이탈). → 무게중심이 가장자리
30px 이내면 배제. 통합 데모에서 배제 357건 동작 확인.

**⑤ 관찰: 이동 탐지가 정확도를 개선**
드론이 여러 각도에서 본 탐지가 누적 평균에 섞이며 높이 편향이 상쇄 →
확정 좌표가 참값으로 수렴 (실측: #1 (-2.15,2.18) → (-2.03,2.11), 참값 (-2,2)).
미션 시작 시점 타겟 좌표 오차 3~11cm.

---

## 4-A. 권장 실행법 — 무대/공연 분리 (터미널 2개) ★

시나리오 조건(탐사 범위·복귀 좌표·속도)을 바꿀 때마다 Gazebo 를 재시작하지
않아도 되는 방식. **평소에는 이 방식을 쓴다.**

### 터미널 1 — 🎭 무대 (한 번만 켜고 계속 둠)

```bash
killall -9 gzserver gzclient 2>/dev/null
pkill -f obstacle_locator; pkill -f spartina_detector
pkill -f target_manager; pkill -f mission_manager; pkill -f drone_survey

export TURTLEBOT3_MODEL=waffle
export ROS_LOCALHOST_ONLY=1
ros2 launch ~/drone_project/launch/spartina_phase4_base.launch.py
```

내용: Gazebo + 로봇 + TF + Nav2 + locator + 탐지기 + target_manager
+ mission_manager. 전부 대기 상태로 진입하며 아무것도 움직이지 않는 게 정상.

### 터미널 2 — 🎬 공연 (조건 바꿔가며 몇 번이든)

```bash
export TURTLEBOT3_MODEL=waffle
export ROS_LOCALHOST_ONLY=1

# 인자 순서: x_min x_max y_min y_max home_x home_y speed hold_sec
bash ~/drone_project/scripts/run_scenario.sh -3 3 -3 3 -3 -3 1.5 1.5
```

| 인자 위치 | 의미 | 담당 | 기본값 |
| --- | --- | --- | --- |
| 1~4 | 탐사 범위 x_min x_max y_min y_max | 🚁 | ±3.0 |
| 5~6 | 복귀 좌표 home_x home_y | 🚗 | -3.0 -3.0 |
| 7 | 비행 속도 [m/s] | 🚁 | 0.8 |
| 8 | 웨이포인트 정지 시간 [s] | 🚁 | 3.0 |

- 뒤에서부터 생략 가능 (생략분은 기본값). 정수로 써도 자동으로 실수 변환.
- 시나리오 흐름: 리셋 → 🚁 지정 범위 1회 탐사 → 이륙 지점 복귀·정지(DONE)
  → 🚗 DONE 확인 후 출발 → 발견된 기둥 순차 방문 → 지정 좌표 복귀 → 완료
- `Ctrl+C` 는 드론 탐사만 중단 (무대는 유지). 끝난 뒤 인자 바꿔 재실행 가능.
- 복귀 좌표 주의: 기둥((2,0),(-2,2),(0,-2))·장애물((0,0),(3,-1.5),(3.5,1.5))과
  1m 이상 떨어진 빈 곳으로.
- 속도 트레이드오프: 속도↑ → 탐사 빨라지나 탐지 산포↑·관측 수↓ →
  확정 좌표 정확도 소폭 저하. 1.5~2.0 권장, 4.0 이상 비권장.

## 4-B. 개별 실행 가이드 (7터미널 — 개발·디버깅용)

### 사전 준비

```bash
# 잔재 청소 (재시작 시 항상 먼저)
killall -9 gzserver gzclient
pkill -f obstacle_locator; pkill -f spartina_detector
pkill -f target_manager; pkill -f mission_manager; pkill -f drone_survey
```

**모든 터미널에서 맨 먼저 이 두 줄:**

```bash
export TURTLEBOT3_MODEL=waffle
export ROS_LOCALHOST_ONLY=1
```

### T1 — Gazebo (multi 월드)

> **뭐 하는 것:** 시뮬레이션 세계 자체. 목표 기둥 3개·장애물 3개·드론 카메라·로봇을
> 띄우고, 드론 카메라 영상(`/drone/image_raw`)과 모델 pose(`/gazebo/model_states`)를
> 발행한다. 런치가 7초 뒤 `drone_tf_broadcaster` 도 자동 실행 — 이게 드론 pose 를
> TF(map→drone_camera_optical)로 변환해 좌표 계산의 재료를 만든다.

```bash
ros2 launch ~/drone_project/launch/spartina_multi.launch.py
```

확인: 빨간 기둥 3개. TF 브로드캐스터는 7초 뒤 자동 시작.

### T2 — Nav2

> **뭐 하는 것:** 로봇의 자율주행 스택. 경로 계획(planner) + 주행 제어(controller)
> + 장애물 회피(costmap)를 담당한다. mission_manager 가 보내는 NavigateToPose
> 액션 goal 을 받아 로봇을 실제로 움직이고, 도착/실패를 회신한다.
> 파라미터는 파트3에서 튜닝한 `nav2_explore.yaml` (max_vel_x 0.45, sim_time 2.2,
> 사각 footprint) 사용.

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=$HOME/drone_project/config/nav2_explore.yaml
```

확인: 에러 반복 없이 안정화. (`ros2 action list | grep navigate` 에
`/navigate_to_pose` 보이면 OK)

### T3 — 좌표 계산 (locator multi)

> **뭐 하는 것:** "화면의 몇 번째 픽셀"(2D)을 "지도 위 어디"(map 절대 좌표)로
> 번역하는 번역기. 탐지 픽셀 방향으로 광선을 쏴 지면(z=0)과의 교차점을 구한다.
> 핵심 규약: 투영에 쓰는 드론 pose 는 "지금"이 아니라 **영상이 촬영된 시각**의
> TF 를 조회한다 (이동 중 0.77m 오차를 없앤 Phase 3 의 핵심).
> 원본(정우열)을 상속해 다중 목표용으로 확장: 탐지 즉시 투영 + 재시도 큐.

```bash
python3 ~/drone_project/scripts/obstacle_locator_multi_node.py --ros-args \
  -p use_tf2:=true -p world_frame:=map \
  -p camera_frame:=drone_camera_optical -p robot_frame:=base_link \
  -p use_sim_time:=true -p lookup_timeout:=0.0
```

★ `lookup_timeout:=0.0` 필수 (§3-② 참고). 확인: `다중 목표 모드 시작`.

### T4 — 탐지기 (multi)

> **뭐 하는 것:** 드론 영상에서 빨간 덩어리(목표)를 HSV 색 검출로 찾아
> 픽셀 좌표를 발행한다. 시뮬용 파이프라인 검증기 — 실전에서는 파트1 의 YOLO 가
> 같은 인터페이스(/detection/pixel, 영상 stamp 유지)로 이 노드를 대체한다.
> 다중판 확장: 한 프레임의 모든 덩어리를 각각 발행 + 화면 가장자리 30px 필터
> (반쯤 잘린 목표의 무게중심 이상치 차단).

```bash
python3 ~/drone_project/scripts/spartina_detector_multi_node.py --ros-args \
  -p use_sim_time:=true -p show_window:=true
```

확인: 탐지 창에 초록 테두리 + `targets in frame: N`.

### T5 — target_manager

> **뭐 하는 것:** 같은 목표가 수백 번 재탐지되는 낱개 좌표 스트림을
> "타겟 N개의 확정 리스트" 로 정리하는 클러스터링 노드.
> 새 좌표가 기존 타겟과 0.5m(merge_radius) 이내면 병합(누적 평균으로 갱신 —
> 관측이 쌓일수록 참값에 수렴), 아니면 새 후보. 관측 3회 이상만 "확정" 승격
> (1회성 오탐 자동 필터). 실측: 1,866건 수신 → 확정 3개, 좌표 오차 3~11cm.

```bash
python3 ~/drone_project/scripts/target_manager_node.py --ros-args -p use_sim_time:=true
```

확인: `새 후보 등록` → `★ 타겟 확정`. 후보가 3개에서 멈춰야 정상.

### T6 — mission_manager (v3: 홈 복귀 지원)

> **뭐 하는 것:** 확정 타겟들을 로봇이 순차 방문하게 하는 미션 지휘관.
> 확정 3개가 모이면 시작 → 매번 로봇 현재 위치에서 가장 가까운 미방문 타겟
> 선택(greedy) → NavigateToPose 액션으로 goal 전송 → 도착 회신 받으면 다음.
> goal 은 기둥 중심이 아니라 0.6m 앞(접근 오프셋 — 기둥에 collision 이 있어
> 중심 도달 불가). 실패 시 재시도 1회 후 skip. 전부 방문하면 홈으로 복귀.

```bash
# ① 기본 — 3곳 방문 후 미션 시작 위치로 자동 복귀
python3 ~/drone_project/scripts/mission_manager_node.py --ros-args -p use_sim_time:=true

# ② 복귀 좌표 직접 지정 (예: (-3,-3) 코너)
python3 ~/drone_project/scripts/mission_manager_node.py --ros-args -p use_sim_time:=true \
  -p use_home_pose:=true -p home_pose:="[-3.0, -3.0]"

# ③ 복귀 없이 마지막 기둥에서 종료
python3 ~/drone_project/scripts/mission_manager_node.py --ros-args -p use_sim_time:=true \
  -p return_home:=false
```

확인: 확정 3개 모이면 `■ 미션 시작 ... | 홈 (x,y) [지정/자동(시작위치)]` → 로봇 출발
→ `✔ 도착 (n/3)` × 3 → `◀ 홈으로 복귀` → `✔ 홈 도착` → `■■ 미션 완료 ... + 홈 복귀`.

※ 복귀 좌표 지정 시 주의: 기둥 3개((2,0), (-2,2), (0,-2))·장애물 3개((0,0),
(3,-1.5), (3.5,1.5))와 **1m 이상 떨어진 빈 곳**, 탐사 영역(±3m) 안으로 지정할 것.
겹치면 Nav2 가 goal 도달 불가로 실패한다.

### T7 — 드론 지그재그 탐사

> **뭐 하는 것:** 드론을 잔디 깎기 패턴으로 영역 전체를 훑게 하는 노드 (정우열, Phase 2).
> 영역을 lane_spacing 간격의 레인으로 잘라 방향을 번갈아 왕복 — 고도 8m 시야
> 폭(±4.6m)이 레인 간격(3m)보다 넓어 빈틈없이 커버된다. 순간이동이 아니라
> 타이머로 매 주기 speed×Δt 만큼 pose 를 옮겨 "연속 이동" 을 만들므로 TF 가
> 부드럽게 갱신됨. 웨이포인트마다 hold_sec 정지(MOVING↔HOLD, /drone/survey_state).

```bash
python3 ~/drone_project/scripts/drone_survey_node.py --ros-args \
  -p use_sim_time:=true \
  -p area_x_min:=-3.0 -p area_x_max:=3.0 \
  -p area_y_min:=-3.0 -p area_y_max:=3.0 \
  -p lane_spacing:=3.0 -p altitude:=8.0 -p speed:=0.8 -p hold_sec:=3.0
```

### 데모 녹화용 실행 순서 (스토리가 살아나는 순서)

1. T1~T4 켜기 (기반만 — **T5는 아직 켜지 말 것**, 정지 상태에서 즉시 확정돼버림)
2. 화면 배치: Gazebo + 탐지 창 + T5·T6 터미널 → `Ctrl+Alt+Shift+R` 녹화 시작
3. **T7 (드론 이륙) → T5 → T6** 순서로 켜기
4. 드론 탐사 → 타겟 확정 쌓임 → 3개 도달 → 로봇 자동 출발 → 3곳 순회 → 미션 완료
5. `Ctrl+Alt+Shift+R` 녹화 종료 → 영상은 `~/Videos` 에 저장

### 자주 터지는 문제

| 증상 | 원인 / 해결 |
| --- | --- |
| Gazebo 검은 화면/안 뜸 | 이전 프로세스 잔재 → `killall -9 gzserver gzclient` 후 재시도 |
| `투영 불가 — TF 없음` 대량 발생 | `lookup_timeout:=0.0` 빠짐 (§3-② 기아 문제) |
| 좌표/TF 가 이상함, extrapolation 에러 | 어떤 노드에 `use_sim_time:=true` 누락 |
| 타겟 후보가 4개 이상 | 이동 산포가 merge_radius 초과 → `-p merge_radius:=0.6` 등으로 조정 (근거 기록) |
| 로봇이 안 움직임 | Nav2 액션 서버 미기동 → `ros2 action list | grep navigate` 확인 |
| 시뮬 시간 리셋 후 노드들 혼란 | Gazebo 재시작 시 파이썬 노드도 전부 재시작할 것 |
| 홈 복귀 실패 (goal 도달 불가) | 지정 홈 좌표가 기둥/장애물과 겹침 → 빈 곳으로 변경. 재시도 초과 시 현 위치 종료가 정상 동작 |
| `expected DOUBLE got INTEGER` 파라미터 에러 | ROS2 파라미터는 타입 엄격 — 정수(-3) 대신 실수(-3.0). run_scenario.sh 는 자동 변환하나, 노드를 직접 실행할 땐 소수점 필수 |
| 공연 2회차에서 이전 타겟이 남아있음 | run_scenario.sh 를 안 거치고 드론만 실행한 경우 — 리셋 토픽 수동 발행: `ros2 topic pub --once /mission/reset std_msgs/msg/Empty "{}"` 및 `/targets/reset` 동일 |

---

## 5. 남은 일 (다음 세션)

- [ ] **Phase 4 검증결과 문서 작성** — 선배 Week5 문서 스타일로, §1·§3 의 실측 수치 활용
- [ ] **선배·멘토님 진행 보고** — 특히 locator 상속 구조와 재시도 큐(§3-②)는 공유 필요
- [ ] 통합 데모 녹화본 확보 (아직이면 §4 데모 순서로)
- [ ] (선택) 7터미널을 런치 파일 하나로 통합
- [ ] (선택) 파트1 YOLO 연동 대비 — 인터페이스 규약은 `spartina_detector_multi_node.py` 상단 주석 참고 (영상 header.stamp 유지가 핵심)
- [ ] (선택) 궤도차량(tracked-robot 브랜치)과의 통합 여부 팀 논의
