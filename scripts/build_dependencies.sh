#!/usr/bin/env bash
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
prefix="$repo_root/deps/install"
cmake -S "$repo_root/deps/Livox-SDK2" -B "$repo_root/deps/build/sdk"   -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"   -DCMAKE_SHARED_LINKER_FLAGS=-Wl,-Bsymbolic
cmake --build "$repo_root/deps/build/sdk" --parallel 2
cmake --install "$repo_root/deps/build/sdk"
cmake -S "$repo_root/deps/spdlog" -B "$repo_root/deps/build/spdlog"   -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"   -DSPDLOG_BUILD_SHARED=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON   -DSPDLOG_BUILD_EXAMPLE=OFF -DSPDLOG_BUILD_TESTS=OFF
cmake --build "$repo_root/deps/build/spdlog" --parallel 2
cmake --install "$repo_root/deps/build/spdlog"
cmake -S "$repo_root/deps/Micro-XRCE-DDS-Agent" -B "$repo_root/deps/build/agent"   -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"   -DCMAKE_PREFIX_PATH=/opt/ros/jazzy -DUAGENT_SUPERBUILD=OFF   -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON   -DUAGENT_USE_SYSTEM_LOGGER=ON -DUAGENT_P2P_PROFILE=OFF   -Dspdlog_DIR="$prefix/lib/cmake/spdlog"   -DUAGENT_BUILD_TESTS=OFF -DUAGENT_BUILD_USAGE_EXAMPLES=OFF   -DCMAKE_INSTALL_RPATH=/opt/ros/jazzy/lib -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
cmake --build "$repo_root/deps/build/agent" --parallel 2
cmake --install "$repo_root/deps/build/agent"
make -C "$repo_root/deps/linuxptp" -j2 ptp4l
