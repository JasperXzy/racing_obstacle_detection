cmake_minimum_required(VERSION 3.8)
project(racing_obstacle_detection)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# 设置自定义路径
set(DNN_PATH "/usr/include/dnn")
set(DNN_LIB_PATH "/usr/lib/")

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${DNN_PATH}
)

# 创建可执行文件
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/parser.cpp
  src/image_utils.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  OpenCV
  sensor_msgs
  hbm_img_msgs
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  dnn
  pthread
  rt
  dl
  nlohmann_json::nlohmann_json
)

# 安装规则
install(DIRECTORY include/
  DESTINATION include
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()