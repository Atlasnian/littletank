cmake_minimum_required(VERSION 3.8)
project(pub_and_sub)

#必要的
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#find_package
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  system    # 基础系统库
  filesystem # 文件系统操作
  thread     # 多线程支持
)

#可执行文件1
add_executable(pub_test_node src/pub_test.cpp)
ament_target_dependencies(pub_test_node  #声明依赖
rclcpp
std_msgs
geometry_msgs
nav_msgs
sensor_msgs
)


#添加install目录(src)
install(TARGETS  pub_test_node
  DESTINATION lib/${PROJECT_NAME})


#必要的
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()