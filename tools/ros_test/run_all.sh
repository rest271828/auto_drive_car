#!/usr/bin/env bash
# 一键测试: 环境检查 -> colcon build -> 离线仿真 -> 三个场景的 ROS 闭环冒烟测试
# 用法(在仓库根目录): bash tools/ros_test/run_all.sh
#   可选环境变量: RVIZ=true 打开RViz看过程; DURATION=40 每个场景测试秒数; SKIP_BUILD=1 跳过编译
# 结果汇总在 test_logs/<时间>/summary.txt, 把它整个贴回来即可
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$ROOT/test_logs/$STAMP"
mkdir -p "$LOG"
SUMMARY="$LOG/summary.txt"
RVIZ="${RVIZ:-false}"
DURATION="${DURATION:-40}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}" # 避免和局域网里其他ROS程序串话

say() { echo "$*" | tee -a "$SUMMARY"; }

say "######## auto_drive_car test $STAMP ########"
say "git: $(git rev-parse --abbrev-ref HEAD) $(git rev-parse --short HEAD) $(git status --porcelain | wc -l) uncommitted files"

# ---------- 1. 环境 ----------
say ""
say "== 1. environment =="
say "os: $(lsb_release -ds 2>/dev/null || uname -a)"
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  say "ROS_DISTRO: ${ROS_DISTRO:-?}"
else
  say "FAIL: /opt/ros/humble/setup.bash not found (ROS 2 Humble 没装?)"
  exit 1
fi
say "gcc: $(gcc -dumpfullversion 2>/dev/null)  cmake: $(cmake --version | head -1 | awk '{print $3}')  python: $(python3 --version 2>&1)"
for p in /usr/local/include/eigen-3.4.0 /usr/include/eigen3 /usr/local/include/OsqpEigen /usr/local/include/osqp; do
  [ -d "$p" ] && say "found $p" || say "missing $p"
done
for pkg in xacro robot_state_publisher joint_state_publisher rviz2 tf2_ros; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 && say "ros pkg ok: $pkg" || say "ros pkg MISSING: $pkg  (sudo apt install ros-humble-${pkg//_/-})"
done

# ---------- 2. 编译 ----------
say ""
say "== 2. colcon build =="
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  if colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release >"$LOG/build.log" 2>&1; then
    say "build: OK ($(grep -c 'warning:' "$LOG/build.log") warnings)"
  else
    say "build: FAILED, first errors:"
    grep -n -E "error|Error|undefined reference|No such file" "$LOG/build.log" | head -40 | tee -a "$SUMMARY"
    say "(full log: $LOG/build.log)"
    exit 1
  fi
else
  say "skipped"
fi
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"

# ---------- 3. 离线仿真 ----------
say ""
say "== 3. offline sim (no ROS) =="
if bash tools/offline_sim/build.sh >"$LOG/offline_build.log" 2>&1; then
  export SIM_SHARE_DIR="$ROOT/src/planning"
  for cfg in static onlane dynamic; do
    SIM_LOG=3 ./tools/offline_sim/offline_sim "planning_${cfg}_obs_config.yaml" "$LOG/offline_${cfg}.csv" 2>"$LOG/offline_${cfg}.err" | tee -a "$SUMMARY"
  done
else
  say "offline sim build FAILED:"
  tail -20 "$LOG/offline_build.log" | tee -a "$SUMMARY"
fi

# ---------- 4. ROS 闭环 ----------
say ""
say "== 4. ROS closed loop (each ${DURATION}s, rviz=$RVIZ) =="
cleanup() {
  pkill -INT -f "planning_launch.py" 2>/dev/null
  sleep 2
  for n in planning_process car_move_cmd obs_move_cmd pnc_map_server global_path_server robot_state_publisher joint_state_publisher rviz2; do
    pkill -f "/$n" 2>/dev/null
  done
  sleep 1
}
trap cleanup EXIT

declare -A MIN_TRAVEL=([static]=150 [onlane]=150 [dynamic]=150)
PASS=0
for cfg in static onlane dynamic; do
  cleanup
  name="planning_${cfg}_obs_config.yaml"
  setsid ros2 launch planning planning_launch.py config:="$name" rviz:="$RVIZ" >"$LOG/launch_${cfg}.log" 2>&1 &
  sleep 4 # 等节点和服务起来
  python3 tools/ros_test/smoke_check.py --config "$name" --duration "$DURATION" \
    --min-travel "${MIN_TRAVEL[$cfg]}" 2>&1 | tee -a "$SUMMARY"
  [ "${PIPESTATUS[0]}" = "0" ] && PASS=$((PASS + 1))
  cleanup
  # 节点日志里的错误/崩溃
  errs="$(grep -E "\[ERROR\]|process has died|exit code|terminate called|Segmentation" "$LOG/launch_${cfg}.log" | head -15)"
  if [ -n "$errs" ]; then
    say "  launch log errors:"
    echo "$errs" | tee -a "$SUMMARY"
  fi
  # 决策日志统计(看决策是否合理)
  grep -oE "path decision: bypass obs [0-9]+ on the [a-z]+|speed decision: obs [0-9]+ -> [A-Z]+" "$LOG/launch_${cfg}.log" \
    | sort | uniq -c | head -10 | sed 's/^/  /' | tee -a "$SUMMARY"
done

say ""
say "######## ROS closed loop: $PASS/3 scenarios passed ########"
say "logs: $LOG"
