cmake_minimum_required(VERSION 3.8)
project(bring_up)

#必要的
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)



#添加install目录(launch和shell脚本)
install(DIRECTORY  launch config shell
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)


#必要的
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
