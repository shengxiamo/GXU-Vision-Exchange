cmake_minimum_required(VERSION 3.8)
project(joint_state_publisher)

# 设置编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行目标
add_executable(joint_state_publisher_node src/joint_state_publisher.cpp)

# 链接依赖库
ament_target_dependencies(joint_state_publisher_node
  rclcpp
  sensor_msgs
  OpenCV
)

# 显式链接 OpenCV 库
target_include_directories(joint_state_publisher_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(joint_state_publisher_node ${OpenCV_LIBS})

# 安装可执行文件
install(TARGETS joint_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# 如果启用测试，加载必要依赖
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 忽略一些代码检查工具（根据需要调整）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 标记为 ROS 2 功能包
ament_package()
