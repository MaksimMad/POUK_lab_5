cmake_minimum_required(VERSION 3.5)
project(simple_map)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Включение директорий с заголовками
include_directories(include)

# Создание библиотеки scan_to_map
add_library(scan_to_map src/scan_to_map.cpp)
target_link_libraries(scan_to_map
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(scan_to_map
  rclcpp
  sensor_msgs
  nav_msgs
)


add_executable(simple_map_node src/simple_map_1.cpp)
target_link_libraries(simple_map_node
  scan_to_map
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(simple_map_node
  rclcpp
  sensor_msgs
  nav_msgs
)



# Установка файлов
install(TARGETS
  scan_to_map
  simple_map_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка заголовочных файлов (если они нужны для других пакетов)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
