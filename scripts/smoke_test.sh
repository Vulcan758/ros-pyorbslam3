#!/usr/bin/env bash
# Start slam_node with a bundled settings file, wait until it is ready, call
# ~/reset, then stop it with SIGINT. Exercises the vocabulary download, the
# settings checks, ORB-SLAM3 start-up and the services without any data.
set -euo pipefail

log=$(mktemp)
# run the installed executable directly: SIGINT to a `ros2 run` wrapper does not reach the node
exe="$(ros2 pkg prefix pyorbslam3_ros)/lib/pyorbslam3_ros/slam_node"
"$exe" --ros-args \
  -p sensor:=stereo_imu -p settings_file:=tumvi_stereo_imu.yaml >"$log" 2>&1 &
pid=$!
trap 'kill -INT $pid 2>/dev/null || true' EXIT

for _ in $(seq 1 600); do
  if grep -q "ready: sensor=stereo_imu" "$log"; then break; fi
  if ! kill -0 $pid 2>/dev/null; then cat "$log"; echo "slam_node exited early"; exit 1; fi
  sleep 1
done
grep -q "ready: sensor=stereo_imu" "$log" || { cat "$log"; echo "not ready after 600 s"; exit 1; }

# capture before grepping: `cmd | grep -q` fails with SIGPIPE (141) under pipefail
# when grep exits on the first match while cmd is still writing
out=$(ros2 service call /orbslam3/reset std_srvs/srv/Trigger)
grep -q "success=True" <<<"$out" || { echo "$out"; echo "reset failed"; exit 1; }
out=$(ros2 service call /orbslam3/save_trajectory pyorbslam3_ros_msgs/srv/SaveTrajectory \
  "{path: /tmp/x.txt, format: kitti, keyframes_only: true}")
grep -q "success=False" <<<"$out" || { echo "$out"; echo "kitti + keyframes_only not rejected"; exit 1; }

kill -INT $pid
for _ in $(seq 1 30); do kill -0 $pid 2>/dev/null || break; sleep 1; done
trap - EXIT
grep -q "shutting down ORB-SLAM3" "$log" || { cat "$log"; echo "no clean shutdown"; exit 1; }
echo "smoke test passed"
