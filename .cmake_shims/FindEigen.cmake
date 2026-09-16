# Shim for legacy `find_package(Eigen REQUIRED)` calls (e.g. LIO-SAM),
# which expect the old ros-*-cmake-modules FindEigen.cmake that ships
# on stock Ubuntu ROS installs but isn't packaged in this conda-based
# ROS2 environment. Delegates to the standard Eigen3 config.
find_package(Eigen3 REQUIRED NO_MODULE)
set(Eigen_FOUND TRUE)
set(EIGEN_FOUND TRUE)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
set(EIGEN_DEFINITIONS "")
