cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 2. pure_pursuit_node 실행 파일 생성
add_executable(pure_pursuit_node src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit_node rclcpp ackermann_msgs nav_msgs tf2)
target_include_directories(pure_pursuit_node PUBLIC include)

# 3. controller_node 실행 파일 생성
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp ackermann_msgs sensor_msgs)
target_include_directories(controller_node PUBLIC include)

# 4. 생성된 모든 타겟(실행파일) 설치
install(TARGETS
  pure_pursuit_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5. launch 및 include 폴더 설치
install(DIRECTORY
  launch
  include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
