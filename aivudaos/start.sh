#!/usr/bin/env bash
set -eo pipefail
: "${AIVUDA_APP_HELPERS_ENTRY_PATH:?AivudaOS helper is required}"
: "${AIVUDA_APP_CONFIG_PATH:?AivudaOS configuration is required}"
source "$AIVUDA_APP_HELPERS_ENTRY_PATH"
ros_setup="$(aivuda_yaml_get ros.setup_script)"
if aivuda_yaml_has fastlio.ros_domain_id; then
  domain="$(aivuda_yaml_get fastlio.ros_domain_id)"
elif aivuda_yaml_has ros.domain_id; then
  domain="$(aivuda_yaml_get ros.domain_id)"
else
  domain="-1"
fi
px4_setup="$(aivuda_yaml_get workspaces.px4_msgs_setup)"
livox_setup="$(aivuda_yaml_get workspaces.livox_setup)"
lio_setup="$(aivuda_yaml_get workspaces.fastlio_setup)"
if [[ -z "$lio_setup" ]]; then
  lio_setup="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/ros_ws/install/local_setup.bash"
fi
: "${AIVUDA_APP_RUNTIME_DATA_PATH:?AivudaOS runtime directory is required}"
counter_dir="$(aivuda_yaml_get state.counter_directory)"
counter_dir="${counter_dir:-$AIVUDA_APP_RUNTIME_DATA_PATH/reset_counter}"
if [[ "$domain" == "-1" ]]; then
  domain="${ROS_DOMAIN_ID:-0}"
fi
[[ "$domain" =~ ^[0-9]+$ ]] && (( 10#$domain <= 232 )) || { echo 'Invalid ROS_DOMAIN_ID' >&2; exit 2; }
export ROS_DOMAIN_ID="$domain"
[[ -n "$ros_setup" && -n "$lio_setup" && "$counter_dir" == /* ]] || { echo 'ROS, FAST-LIO, and absolute counter directory paths are required.' >&2; exit 2; }
for setup in "$ros_setup" "$px4_setup" "$livox_setup" "$lio_setup"; do
  [[ -n "$setup" ]] || continue
  [[ "$setup" == /* && -f "$setup" && -r "$setup" ]] || { echo "ROS setup file unavailable: $setup" >&2; exit 1; }
  source "$setup"
done
for pkg in fast_lio fastlio_px4_bridge livox_ros_driver2 px4_msgs; do
  ros2 pkg prefix "$pkg" >/dev/null || { echo "ROS package missing: $pkg" >&2; exit 1; }
done
app_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
transport_mode=localhost_udp
if aivuda_yaml_has fastlio.transport_mode; then
  transport_mode="$(aivuda_yaml_get fastlio.transport_mode)"
fi
case "$transport_mode" in
  localhost_udp)
    exec bash "$app_root/ros_ws/install/fastlio_px4_bridge/share/fastlio_px4_bridge/launch/with_localhost_udp.sh" \
      python3 "$app_root/scripts/fusion_guard.py" "$counter_dir" ;;
  inherit) exec python3 "$app_root/scripts/fusion_guard.py" "$counter_dir" ;;
  *) echo "Unknown fastlio.transport_mode: $transport_mode" >&2; exit 2 ;;
esac
