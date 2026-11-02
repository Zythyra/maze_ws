cmake_minimum_required(VERSION 3.8)
project(maze_wall_pid)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_executable(left2_lidar src/left2_lidar.cpp)
target_compile_features(left2_lidar PUBLIC cxx_std_17)
target_include_directories(left2_lidar PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(left2_lidar rclcpp geometry_msgs sensor_msgs nav_msgs
  std_msgs std_srvs visualization_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(left2_lidar ${OpenCV_LIBS})

install(TARGETS left2_lidar DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch launch_support config rviz world DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/teleop_keyboard.py scripts/velocity_guard.py
  scripts/maze_odometry.py scripts/wall_start.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/terminal_start.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_left2_trace test/test_left2_trace.cpp)
  target_compile_features(test_left2_trace PUBLIC cxx_std_17)
  target_include_directories(test_left2_trace PRIVATE include)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/turtlebot3_model.dsv.in")
ament_package()
