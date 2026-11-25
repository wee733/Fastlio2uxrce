#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$(dirname -- "$ROOT")}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
NAME="$(python3 - "$ROOT/manifest.yaml" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    manifest = yaml.safe_load(stream)
print(f"{manifest['app_id']}-{manifest['version']}.tar.gz")
PY
)"
[[ -r "$ROOT/ros_ws/install/local_setup.bash" && -x "$ROOT/ros_ws/install/fastlio_px4_bridge/lib/fastlio_px4_bridge/fastlio_to_px4_visual_odometry" ]] || {
  echo 'Packaged ROS install is missing. Stage the built fast_lio and fastlio_px4_bridge packages in ros_ws/install.' >&2
  exit 1
}
target="$(mktemp "$OUT/.$NAME.XXXXXX")"
trap 'rm -f "$target"' EXIT
tar -czf "$target" -C "$ROOT" --exclude='__pycache__' --exclude='.pytest_cache' manifest.yaml README.md icon.png start.sh pack.sh config scripts tests ros_ws/src ros_ws/install
chmod 644 "$target"
mv -f "$target" "$OUT/$NAME"
echo "$OUT/$NAME"
