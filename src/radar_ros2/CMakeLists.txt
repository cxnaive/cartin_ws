cmake_minimum_required(VERSION 3.8)
project(radar_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# OpenCV
find_package(OpenCV 4 REQUIRED)
set(link ${OpenCV_LIBS} ${link})

# Radar SDK
set(include ${Boost_LIBRARIES_DIRS} ${include})
set(include OculiiSDK/include ${include})
set(include OculiiSDK/include/oculii_core/radar/include ${include})
set(include OculiiSDK/include/oculii_core/imu/include ${include})
set(include OculiiSDK/include/oculii_core/camera/include ${include})
set(include OculiiSDK/tracker/include ${include})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
