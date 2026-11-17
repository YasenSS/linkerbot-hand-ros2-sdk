#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: bash tests/cpp/verify.sh HAND_CPP_SOURCE TRANSPORT_CPP_SOURCE [RESULTS_DIR]" >&2
  exit 2
fi
: "${ROS_DISTRO:?Source the intended ROS distribution first}"
hand_source=$(realpath "${1}")
transport_source=$(realpath "${2}")
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
results_dir=${3:-"${repo_root}/reports/cpp-${ROS_DISTRO}"}
sanitizers=${LINKERBOT_ROS_CPP_SANITIZERS:-OFF}
mkdir -p "$results_dir"
results_dir=$(realpath "$results_dir")
for source_dir in "$hand_source" "$transport_source"; do
  if [[ ! -f "$source_dir/CMakeLists.txt" ]]; then
    echo "Missing CMakeLists.txt in $source_dir" >&2
    exit 2
  fi
done

export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
export ROS_LOG_DIR="${results_dir}/ros-logs"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89}
export ROS_LOCALHOST_ONLY=1

colcon --log-base "$results_dir/build-log" build \
  --base-paths "$repo_root/linkerbot_hand_msgs" "$repo_root/linkerbot_hand_ros_cpp" \
  --packages-up-to linkerbot_hand_ros_cpp \
  --build-base "$results_dir/build" --install-base "$results_dir/install" \
  --cmake-args "-DLINKERBOT_HAND_SOURCE_DIR=$hand_source" \
    "-DLINKERBOT_COMM_SOURCE_DIR=$transport_source" \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DLINKERBOT_ROS_CPP_WARNINGS_AS_ERRORS=ON \
    "-DLINKERBOT_ROS_CPP_SANITIZERS=$sanitizers"
# shellcheck source=/dev/null
source "$results_dir/install/setup.bash"
# Match the SDK sanitizer policy: fail on ASan/UBSan errors; leak checks are separate.
if [[ "$sanitizers" == ON ]]; then
  export ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0:halt_on_error=1:strict_string_checks=1}
  export UBSAN_OPTIONS=${UBSAN_OPTIONS:-print_stacktrace=1:halt_on_error=1}
fi
colcon --log-base "$results_dir/test-log" test \
  --base-paths "$repo_root/linkerbot_hand_msgs" "$repo_root/linkerbot_hand_ros_cpp" \
  --packages-select linkerbot_hand_ros_cpp \
  --build-base "$results_dir/build" --install-base "$results_dir/install" \
  --return-code-on-test-failure
colcon test-result --test-result-base "$results_dir/build/linkerbot_hand_ros_cpp" --verbose
