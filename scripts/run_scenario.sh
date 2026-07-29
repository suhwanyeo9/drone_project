#!/bin/bash
# ══════════════════════════════════════════════════════════════════
# 🎬 [공연 스크립트] run_scenario.sh — 무대 위에서 시나리오 실행/재실행
# ══════════════════════════════════════════════════════════════════
# 전제: 무대 런치(spartina_phase4_base.launch.py)가 떠 있는 상태.
# 하는 일 (순서대로):
#   1. 이전 시나리오 흔적 정리 — 남아 있는 드론 탐사 노드 종료,
#      🚗 미션 리셋(/mission/reset), 타겟 리셋(/targets/reset)
#   2. 🚗 자동차 복귀 좌표를 런타임 파라미터로 갱신 (ros2 param set)
#   3. 드론을 이륙 지점(0,0,8)으로 순간 복귀 (매 공연 같은 조건에서 시작)
#   4. 🚁 드론 1회 탐사 실행 (범위·속도 인자 반영) — 포그라운드로 붙어서
#      로그가 이 터미널에 흐른다. 탐사 완료(DONE) 후 자동으로:
#      🚗 로봇이 순차 방문 → 지정 좌표 복귀 (로그는 무대 터미널에 표시)
#
# 사용:
#   bash ~/drone_project/scripts/run_scenario.sh \
#     [x_min] [x_max] [y_min] [y_max] [home_x] [home_y] [speed] [hold_sec]
#
#   # 전부 기본값 (탐사 ±3m, 복귀 (-3,-3), 속도 0.8):
#   bash ~/drone_project/scripts/run_scenario.sh
#
#   # 범위·복귀·속도 지정:
#   bash ~/drone_project/scripts/run_scenario.sh -3 3 -3 3 -3 -3 1.5 1.5
#
# 종료: Ctrl+C 는 드론 탐사만 중단한다 (무대는 그대로).
# 담당: 여수환 · 스마트해운물류 x ICT 멘토링 (파트 3) · Phase 4
# ══════════════════════════════════════════════════════════════════
set -u
export ROS_LOCALHOST_ONLY=1

X_MIN="${1:--3.0}"; X_MAX="${2:-3.0}"
Y_MIN="${3:--3.0}"; Y_MAX="${4:-3.0}"
HOME_X="${5:--3.0}"; HOME_Y="${6:--3.0}"
SPEED="${7:-0.8}"; HOLD="${8:-3.0}"
# 정수 입력(-3)을 실수(-3.0)로 자동 변환 — ROS2 파라미터는 DOUBLE 타입을 요구
to_float() { case "$1" in *.*) echo "$1";; *) echo "$1.0";; esac }
X_MIN=$(to_float $X_MIN);  X_MAX=$(to_float $X_MAX)
Y_MIN=$(to_float $Y_MIN);  Y_MAX=$(to_float $Y_MAX)
HOME_X=$(to_float $HOME_X); HOME_Y=$(to_float $HOME_Y)
SPEED=$(to_float $SPEED);  HOLD=$(to_float $HOLD)

echo "══════════════════════════════════════════════"
echo "🎬 시나리오 시작"
echo "   🚁 탐사 범위 x[$X_MIN, $X_MAX] y[$Y_MIN, $Y_MAX], 속도 $SPEED, 정지 ${HOLD}s"
echo "   🚗 자동차 복귀 좌표 ($HOME_X, $HOME_Y)"
echo "══════════════════════════════════════════════"

# 1) 이전 시나리오 정리 ─────────────────────────
pkill -f drone_survey_once_node 2>/dev/null && sleep 1
echo "[1/4] 리셋 신호 전송 (미션·타겟 초기화)"
ros2 topic pub --once /mission/reset std_msgs/msg/Empty "{}" > /dev/null
ros2 topic pub --once /targets/reset std_msgs/msg/Empty "{}" > /dev/null

# 2) 🚗 자동차 복귀 좌표 런타임 갱신 ─────────────
echo "[2/4] 🚗 복귀 좌표 갱신 → ($HOME_X, $HOME_Y)"
ros2 param set /mission_manager_node home_pose "[$HOME_X, $HOME_Y]" > /dev/null
ros2 param set /mission_manager_node use_home_pose true > /dev/null

# 3) 드론 이륙 지점 복귀 ────────────────────────
echo "[3/4] 🚁 드론을 이륙 지점 (0, 0, 8) 로 복귀"
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
"{state: {name: 'drone_camera', pose: {position: {x: 0.0, y: 0.0, z: 8.0}, \
orientation: {x: 0.0, y: 0.7071068, z: 0.0, w: 0.7071068}}, \
reference_frame: 'world'}}" > /dev/null
sleep 1

# 4) 🚁 드론 1회 탐사 실행 ──────────────────────
echo "[4/4] 🚁 드론 탐사 출발 — 완료(DONE) 후 🚗 로봇이 자동 출발합니다"
echo "      (로봇·타겟 로그는 무대 터미널에서 확인)"
exec python3 "$HOME/drone_project/scripts/drone_survey_once_node.py" --ros-args \
  -p use_sim_time:=true \
  -p area_x_min:=$X_MIN -p area_x_max:=$X_MAX \
  -p area_y_min:=$Y_MIN -p area_y_max:=$Y_MAX \
  -p lane_spacing:=3.0 -p altitude:=8.0 \
  -p speed:=$SPEED -p hold_sec:=$HOLD
