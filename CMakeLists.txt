cmake_minimum_required(VERSION 3.11)
project(lidar_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_SUBMODULE "Use the submoduled libicp instead of the installed version" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)

if(USE_SUBMODULE)
  option(LIB_ONLY "" ON)
  add_subdirectory(icp)
else()
  find_package(CevIcp REQUIRED)
endif()

add_executable(lidar_odometry src/lidar_odometry.cpp)
target_include_directories(lidar_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lidar_odometry cevicp Eigen3::Eigen)
target_compile_features(lidar_odometry PUBLIC c_std_11 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(lidar_odometry rclcpp sensor_msgs nav_msgs tf2 tf2_eigen message_filters)

install(TARGETS lidar_odometry
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

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
