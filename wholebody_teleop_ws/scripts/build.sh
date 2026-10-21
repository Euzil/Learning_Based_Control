BUILD_TYPE="Release"
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE