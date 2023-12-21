cmake_minimum_required(VERSION 3.8)
project(pcd_proj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_definitions(-D__STDC_FORMAT_MACROS)
add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV 4 REQUIRED)
set(link ${OpenCV_LIBS} ${link})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(link ${PCL_LIBRARIES} ${link})

# find ament_cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# find src
file(GLOB_RECURSE src "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${src})
target_link_libraries(${PROJECT_NAME} ${link})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pcd_proj::PointcloudProjectionNode
  EXECUTABLE ${PROJECT_NAME}_node
)


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

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
