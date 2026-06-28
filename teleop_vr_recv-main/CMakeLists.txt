cmake_minimum_required(VERSION 3.8)
project(teleop_vr_recv)

# ROS 2 Humble can pick up a Conda interpreter from PATH during CMake
# configuration, which breaks ament helper scripts when ROS Python modules
# such as catkin_pkg, em, or numpy are missing from that environment.
if(EXISTS "/usr/bin/python3")
  if(NOT DEFINED Python3_EXECUTABLE OR Python3_EXECUTABLE MATCHES "/miniconda|/conda")
    set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
      "System Python used for ROS CMake configuration" FORCE)
  endif()

  if(NOT DEFINED PYTHON_EXECUTABLE OR PYTHON_EXECUTABLE MATCHES "/miniconda|/conda")
    set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
      "System Python used for ROS legacy Python discovery" FORCE)
  endif()
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 查找第三方库依赖
if(EXISTS "/usr/local/gflags/lib/cmake/gflags")
  set(gflags_DIR "/usr/local/gflags/lib/cmake/gflags")
endif()
if(EXISTS "/usr/local/glog/lib/cmake/glog")
  set(glog_DIR "/usr/local/glog/lib/cmake/glog")
endif()
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)

set(GFLAGS_TARGET gflags)
if(TARGET gflags::gflags)
  set(GFLAGS_TARGET gflags::gflags)
endif()

set(GLOG_TARGET glog)
if(TARGET glog::glog)
  set(GLOG_TARGET glog::glog)
endif()

# 指定 toml11 的 CMake 配置文件路径
if(EXISTS "/usr/local/toml11/lib/cmake/toml11")
  set(toml11_DIR "/usr/local/toml11/lib/cmake/toml11")
elseif(EXISTS "/usr/local/lib/cmake/toml11")
  set(toml11_DIR "/usr/local/lib/cmake/toml11")
endif()
find_package(toml11 REQUIRED)

# utils源文件
set(UTILS_SOURCES
    utils/log/log_config.cpp
)

# 源文件
set(SOURCES
    src/teleop_node_main.cpp
    src/teleop_node.cpp
    src/udp_receiver.cpp
    src/vr_data_parser.cpp
    src/vr_coordinate_transformer.cpp
    src/vr_smoothing_pipeline.cpp
    src/vr_button_handler.cpp
    src/workspace_limiter.cpp
    ${UTILS_SOURCES}
)

# 创建可执行文件 - 主节点
add_executable(teleop_vr_recv_node ${SOURCES})

# 头文件目录
target_include_directories(teleop_vr_recv_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

if(TARGET glog::glog)
  get_target_property(GLOG_INTERFACE_INCLUDE_DIRS glog::glog INTERFACE_INCLUDE_DIRECTORIES)
  if(GLOG_INTERFACE_INCLUDE_DIRS)
    target_include_directories(teleop_vr_recv_node PUBLIC ${GLOG_INTERFACE_INCLUDE_DIRS})
  endif()
endif()

# ROS2依赖
ament_target_dependencies(teleop_vr_recv_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    ament_index_cpp
)

# 链接第三方库
target_link_libraries(teleop_vr_recv_node
    ${GLOG_TARGET}
    ${GFLAGS_TARGET}
    toml11::toml11
    Eigen3::Eigen
)

# C++标准
target_compile_features(teleop_vr_recv_node PUBLIC cxx_std_17)

set_target_properties(teleop_vr_recv_node PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "/usr/local/glog/lib:/usr/local/gflags/lib"
    LINK_FLAGS "-Wl,--disable-new-dtags"
)

# 安装可执行文件
install(TARGETS teleop_vr_recv_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# 安装log目录
install(DIRECTORY log/
    DESTINATION share/${PROJECT_NAME}/log
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
