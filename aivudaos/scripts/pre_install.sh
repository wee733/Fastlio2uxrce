#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/install_log.sh"
install_log_init 'FAST-LIO PX4'
install_step 'Check Python dependencies'
command -v python3 >/dev/null || { echo 'Python 3 is required.' >&2; exit 1; }
python3 -c 'import yaml, jsonschema' || { echo 'PyYAML and jsonschema are required (python3-yaml, python3-jsonschema).' >&2; exit 1; }
install_step 'Check runtime requirements'
echo '[Info] This app includes the Jetson ROS 2 Jazzy FAST-LIO and PX4 bridge overlay. Provide ROS 2, Livox, and compatible PX4 messages.'
echo '[Info] Review workspace paths and prerequisites in the app configuration and README.'
install_done
