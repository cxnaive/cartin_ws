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

#Set boost variable
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Threads REQUIRED)
set(link ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${link})

#Sdk
set(include ${Boost_LIBRARIES_DIRS} ${include})
set(include oculii_sdk/include ${include})
set(include oculii_sdk/include/oculii_core/radar/include ${include})
set(include oculii_sdk/include/oculii_core/imu/include ${include})
set(include oculii_sdk/include/oculii_core/camera/include ${include})
set(include oculii_sdk/include/tracker/include ${include})
set(link -lradar_shared -limu_shared -lcamera_shared -loculii_feat_shared -lTrackerLib ${link})



install(
  DIRECTORY oculii_sdk/lib/
  DESTINATION lib
)

#find source
file(GLOB_RECURSE src
    "src/**"
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${src})
target_include_directories(${PROJECT_NAME} PUBLIC include ${include})
target_link_directories(${PROJECT_NAME} PUBLIC oculii_sdk/lib)
target_link_libraries(${PROJECT_NAME} ${link})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN radar_ros2::RadarNode
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
