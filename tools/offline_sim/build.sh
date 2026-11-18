#!/usr/bin/env bash
# 离线仿真编译: 只需要 g++ (C++17), 不需要 ROS / Eigen / OSQP
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../../src/planning/src"
# reference_line_creator 用引号包含 reference_line_smoother.h(会先在同目录找到真实的OSQP版本),
# 所以把它拷到临时目录, 让同目录下是空实现的替身
TMP="$HERE/.build"
mkdir -p "$TMP"
cp "$SRC/reference_line/reference_line_creator.h" "$SRC/reference_line/reference_line_creator.cpp" "$TMP/"
cp "$HERE/stubs/reference_line/reference_line_smoother.h" "$TMP/"
g++ -std=c++17 -O2 -Wall -Wextra \
  -I"$HERE/stubs" -I"$TMP" \
  -I"$SRC/common/config_reader" -I"$SRC/common/math" -I"$SRC/decision_center" \
  -I"$SRC/local_planner" -I"$SRC/local_planner/local_path" -I"$SRC/local_planner/local_speeds" \
  -I"$SRC/vehicle_info" -I"$SRC/vehicle_info/main_car" -I"$SRC/vehicle_info/obs_car" \
  "$HERE/sim_main.cpp" \
  "$SRC/common/config_reader/config_reader.cpp" \
  "$SRC/common/math/curve.cpp" "$SRC/common/math/polynomial_curve.cpp" \
  "$SRC/vehicle_info/vehicle_info_base.cpp" "$SRC/vehicle_info/main_car/main_car_info.cpp" "$SRC/vehicle_info/obs_car/obs_car_info.cpp" \
  "$TMP/reference_line_creator.cpp" \
  "$SRC/decision_center/decision_center.cpp" \
  "$SRC/local_planner/local_path/local_path_planner.cpp" "$SRC/local_planner/local_path/local_path_smoother.cpp" \
  "$SRC/local_planner/local_speeds/local_speeds_planner.cpp" "$SRC/local_planner/local_speeds/local_speeds_smoother.cpp" \
  "$SRC/local_planner/local_trajectory_combiner.cpp" \
  -o "$HERE/offline_sim"
echo "built $HERE/offline_sim"
