cmake_minimum_required(VERSION 3.5)
project(imu_mocap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuData.msg"
  "msg/ImuDataArray.msg"
  "msg/ImuOrientation.msg"
  "msg/ImuOrientationArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

# 编译 mocap_imu_calibration_node
add_executable(mocap_imu_calibration_node
  src/mocap_imu_calibration.cpp
  src/mocap_imu_calibration_node.cpp
  src/json_parser.cpp
)
target_include_directories(mocap_imu_calibration_node PRIVATE include)
ament_target_dependencies(mocap_imu_calibration_node rclcpp std_msgs builtin_interfaces)
target_link_libraries(mocap_imu_calibration_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# 编译 ahrs_orientation_node
add_executable(ahrs_orientation_node
  src/ahrs_orientation.cpp
  src/ahrs_orientation_node.cpp
)
target_include_directories(ahrs_orientation_node PRIVATE include)
ament_target_dependencies(ahrs_orientation_node rclcpp std_msgs builtin_interfaces)
target_link_libraries(ahrs_orientation_node
  Eigen3::Eigen
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 编译 madgwick_sim_test (Madgwick 滤波器仿真测试，对比仿真真值)
add_executable(madgwick_sim_test
  test/madgwick_sim_test.cpp
  src/ahrs_orientation.cpp
)
target_include_directories(madgwick_sim_test PRIVATE include)
ament_target_dependencies(madgwick_sim_test rclcpp std_msgs builtin_interfaces geometry_msgs)
target_link_libraries(madgwick_sim_test
  Eigen3::Eigen
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 安装可执行文件
install(TARGETS
  mocap_imu_calibration_node
  ahrs_orientation_node
  madgwick_sim_test
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 参数配置文件
install(DIRECTORY
  config
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
