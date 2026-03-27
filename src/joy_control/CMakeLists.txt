cmake_minimum_required(VERSION 3.8)
project(joy_control)

#必要的
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#find_package
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)  # 用于获取Joy消息类型
#可执行文件、声明依赖
add_executable(joy_control_node src/joy_control.cpp)
ament_target_dependencies(joy_control_node  #声明依赖
rclcpp 
std_msgs
geometry_msgs
joy
sensor_msgs
)

#添加install目录(launch)
install(DIRECTORY  launch
  DESTINATION share/${PROJECT_NAME}
)

#添加install目录(src)
install(TARGETS  joy_control_node
  DESTINATION lib/${PROJECT_NAME})



#必要的
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
