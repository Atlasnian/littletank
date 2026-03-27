cmake_minimum_required(VERSION 3.8)
project(ira_laser_tools)


#必要的
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#find_package
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rcl_interfaces REQUIRED)

#remove_compile_definitions(DISABLE_LIBUSB_1_0)


#可执行文件、声明依赖
add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger  #声明依赖
rclcpp 
laser_geometry
sensor_msgs
std_msgs
tf2
tf2_ros
pcl_ros
Eigen3
PCL
rcl_interfaces
)


#添加依赖库
target_link_libraries(laserscan_multi_merger
  ${rclcpp_LIBRARIES} #这个可以不写
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}  # 链接 Boost 库
)

#头文件索引包含。ROS2默认索引本功能包内的include文件夹里头文件、系统头文件(包括boost库)和第三方库头文件
include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


#添加install目录(launch)
install(DIRECTORY  launch
  DESTINATION share/${PROJECT_NAME}
)

#添加install目录(src)
install(TARGETS  laserscan_multi_merger
  DESTINATION lib/${PROJECT_NAME})

#必要的
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()