#!/bin/bash
set -e

script_dir="$(cd "$(dirname "$0")" && pwd)"
workspace_dir="$(cd "${script_dir}/../../.." && pwd)"

cd "${workspace_dir}"
colcon build --packages-select livox_ros_driver2 --symlink-install
