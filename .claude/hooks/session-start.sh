#!/bin/bash
# Sets up a ROS2 Humble toolchain for this workspace on Claude Code on the web.
#
# The README targets Ubuntu 22.04 + apt (ros-humble-*), which this container
# (Ubuntu 24.04) can't satisfy directly -- there are no Humble apt binaries
# for Noble. Instead this installs ROS2 Humble via RoboStack (a conda/mamba
# distribution that isn't tied to the host OS version), then builds the
# packages that don't require the Clearpath simulator stack or a GPU/display
# (Gazebo Ignition itself is out of scope here -- see README for that).
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

ROOT_PREFIX="$HOME/.local/share/mamba"
ENV_NAME="ros_humble"
ENV_DIR="$ROOT_PREFIX/envs/$ENV_NAME"
MM_BIN="$HOME/micromamba/micromamba"

if [ ! -x "$MM_BIN" ]; then
  mkdir -p "$HOME/micromamba"
  curl -sS -L https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64 -o "$MM_BIN"
  chmod +x "$MM_BIN"
fi

if [ ! -d "$ENV_DIR" ]; then
  "$MM_BIN" create -y -r "$ROOT_PREFIX" -n "$ENV_NAME" \
    -c robostack-humble -c conda-forge -c robostack-staging \
    ros-humble-ros-base \
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-ros-gz \
    gtsam \
    compilers \
    "cmake=3.26.*" \
    make \
    pkg-config \
    colcon-common-extensions
fi

export PATH="$ENV_DIR/bin:$PATH"
export MAMBA_ROOT_PREFIX="$ROOT_PREFIX"

# Livox-SDK2 is a plain CMake C++ library that livox_ros_driver2 links
# against via find_library(... liblivox_lidar_sdk_shared.so /usr/local/lib).
# Build and install it once per container.
if [ ! -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
  cd "$CLAUDE_PROJECT_DIR/src/Livox-SDK2"
  # Upstream source is missing <cstdint>, which newer GCC no longer
  # pulls in transitively. Patch it in if not already applied.
  if ! grep -q '#include <cstdint>' sdk_core/logger_handler/file_manager.h; then
    sed -i '0,/#include <string>/s//#include <cstdint>\n#include <string>/' sdk_core/logger_handler/file_manager.h
  fi
  mkdir -p build && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  make -j"$(nproc)"
  make install
  ldconfig || true
fi

# livox_ros_driver2 ships separate ROS1/ROS2 package.xml + launch dirs and
# expects its own build.sh to pick the right ones (package.xml is gitignored
# there because it's generated). Do the same substitution non-destructively.
cd "$CLAUDE_PROJECT_DIR/src/livox_ros_driver2"
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

cd "$CLAUDE_PROJECT_DIR"
git submodule update --init --recursive

set +u
source "$ENV_DIR/setup.bash"
set -u
colcon build \
  --packages-skip warthog_gazebo husky_description warthog_description \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_MODULE_PATH="$CLAUDE_PROJECT_DIR/.cmake_shims"

# husky_description / warthog_description / warthog_gazebo depend on
# Clearpath's clearpath_platform_description and ros_gz_sim packages that
# aren't published on RoboStack; their CMakeLists don't call find_package on
# them so a plain build still succeeds, it just can't be rosdep-verified here.
colcon build --packages-select warthog_gazebo husky_description warthog_description

{
  echo "export MAMBA_ROOT_PREFIX=\"$ROOT_PREFIX\""
  echo "export PATH=\"$ENV_DIR/bin:\$PATH\""
  echo "_had_nounset=\$(set +o | grep -c 'set -o nounset' || true)"
  echo "set +u"
  echo "source \"$ENV_DIR/setup.bash\""
  echo "source \"$CLAUDE_PROJECT_DIR/install/setup.bash\" 2>/dev/null || true"
  echo "[ \"\$_had_nounset\" = \"1\" ] && set -u"
  echo "unset _had_nounset"
} >> "$CLAUDE_ENV_FILE"
