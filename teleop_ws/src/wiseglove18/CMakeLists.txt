cmake_minimum_required(VERSION 3.8)
project(wiseglove18)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# WiseGlove SDK 所在目录
set(WISEGLOVE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/include/wiseglove18"
)

# 查找项目中自带的 libwiseglove.so
find_library(
  WISEGLOVE_LIB
  NAMES wiseglove
  PATHS "${WISEGLOVE_DIR}"
  NO_DEFAULT_PATH
)

if(NOT WISEGLOVE_LIB)
  message(FATAL_ERROR
    "wiseglove library not found in ${WISEGLOVE_DIR}"
  )
endif()

message(STATUS "Found WiseGlove library: ${WISEGLOVE_LIB}")

add_executable(
  glove_data_pub
  src/glove_data_pub.cpp
)

target_include_directories(
  glove_data_pub
  PRIVATE
  "${WISEGLOVE_DIR}"
)

ament_target_dependencies(
  glove_data_pub
  rclcpp
  std_msgs
)

target_link_libraries(
  glove_data_pub
  "${WISEGLOVE_LIB}"
)

# 编译目录中运行时，使系统能找到项目里的动态库
set_target_properties(
  glove_data_pub
  PROPERTIES
  BUILD_RPATH "${WISEGLOVE_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

install(
  TARGETS glove_data_pub
  DESTINATION lib/${PROJECT_NAME}
)

# 将动态库和可执行文件安装到同一个目录
install(
  FILES "${WISEGLOVE_DIR}/libwiseglove.so"
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()