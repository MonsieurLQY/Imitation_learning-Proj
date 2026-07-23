cmake_minimum_required(VERSION 3.8)
project(mpc_interpolation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
# find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)
find_package(Boost REQUIRED COMPONENTS system thread atomic)

add_executable(mpc_interpolation_node src/interpolation.cpp
                                      src/mpc_spline.cpp
)

add_executable(cubic_spline_node src/cubic_spline.cpp
                                 src/mpc_spline.cpp
)

target_include_directories(mpc_interpolation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "$ENV{HOME}/.local/qpoases/include"
)
target_include_directories(cubic_spline_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "$ENV{HOME}/.local/qpoases/include"
)
  
target_compile_features(mpc_interpolation_node PUBLIC c_std_99 cxx_std_20)
target_compile_features(cubic_spline_node PUBLIC c_std_99 cxx_std_20)

target_link_libraries(mpc_interpolation_node
  "$ENV{HOME}/.local/qpoases/lib/libqpOASES.so"
  Eigen3::Eigen
  Boost::system
  Boost::thread
  Boost::atomic
)

target_link_libraries(cubic_spline_node
  "$ENV{HOME}/.local/qpoases/lib/libqpOASES.so"
  Eigen3::Eigen
)

ament_target_dependencies(mpc_interpolation_node
  "rclcpp"
  # "Eigen3"
  # "qpOASES"
  "std_msgs"
  "sensor_msgs"
)
ament_target_dependencies(cubic_spline_node
  # "Eigen3"
  # "qpOASES"
  "std_msgs"
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS mpc_interpolation_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS cubic_spline_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()