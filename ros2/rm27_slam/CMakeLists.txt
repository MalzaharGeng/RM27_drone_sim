cmake_minimum_required(VERSION 3.16)
project(rm27_slam CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ros2 run 时能找到 ORB_SLAM3 / DBoW2 / g2o 的 .so（安装后自动写入 RPATH）
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# ---- ORB_SLAM3（源码在 $HOME/ORB_SLAM3，已编译） ----
if(NOT EXISTS "$ENV{HOME}/ORB_SLAM3/include/System.h")
  message(FATAL_ERROR "未找到 $ENV{HOME}/ORB_SLAM3/include/System.h，请先编译 ORB_SLAM3")
endif()
set(ORB_ROOT     "$ENV{HOME}/ORB_SLAM3")
set(ORB_LIB_DIR  "${ORB_ROOT}/build/lib")
set(ORB_G2O_DIR  "${ORB_ROOT}/build/Thirdparty/g2o/lib")

foreach(_lib "${ORB_LIB_DIR}/libORB_SLAM3.so" "${ORB_LIB_DIR}/libDBoW2.so" "${ORB_G2O_DIR}/libg2o.so")
  if(NOT EXISTS "${_lib}")
    message(FATAL_ERROR "缺少库 ${_lib}，请先执行 ~/drone_sim/scripts/build_orbslam3.sh")
  endif()
endforeach()

# Pangolin 0.9.x 不导出 Pangolin_INCLUDE_DIRS；这里只需要头文件
# （System.h -> Viewer.h -> pangolin/pangolin.h），库由 libORB_SLAM3.so 自带。
set(PANGOLIN_INCLUDE_DIR "/usr/local/include" CACHE PATH "Pangolin include dir")

message(STATUS "ORB_SLAM3 root: ${ORB_ROOT}")
message(STATUS "ORB_SLAM3 lib : ${ORB_LIB_DIR}")

add_executable(stereo_slam_node src/stereo_slam_node.cpp)

target_include_directories(stereo_slam_node PRIVATE
  # ORB_SLAM3 的头文件内部用 "Thirdparty/DBoW2/DBoW2/..." 这种
  # 相对源码根的路径互相引用，所以根目录本身必须在 include 路径里
  ${ORB_ROOT}
  ${ORB_ROOT}/include
  ${ORB_ROOT}/include/CameraModels
  ${ORB_ROOT}/Thirdparty
  ${ORB_ROOT}/Thirdparty/Sophus
  ${ORB_ROOT}/Thirdparty/g2o
  ${ORB_ROOT}/Thirdparty/DBoW2
  ${OpenCV_INCLUDE_DIRS}
  ${PANGOLIN_INCLUDE_DIR}
)

# Pangolin 0.9.6 用 epoxy 装载 OpenGL（不是 GLEW）。它的头文件靠
# glplatform.h: `#elif defined(HAVE_EPOXY) -> #include <epoxy/gl.h>`，
# 没有这个宏就只包含老式 <GL/gl.h>，glUniform1d / glBindAttribLocation
# 等现代函数全部"未声明"。这些宏原本由 pango_opengl 目标的
# INTERFACE_COMPILE_DEFINITIONS 传播，我们绕过目标直接包含头文件，
# 所以必须手动补上（与 Pangolin 自身编译时的定义保持一致）。
target_compile_definitions(stereo_slam_node PRIVATE HAVE_EIGEN HAVE_EPOXY)

# 头文件里的若干 inline 函数会调用 epoxy 符号，链接上更保险
find_library(EPOXY_LIBRARY NAMES epoxy)
if(EPOXY_LIBRARY)
  message(STATUS "libepoxy: ${EPOXY_LIBRARY}")
else()
  message(WARNING "未找到 libepoxy，若运行期报 epoxy 符号缺失请安装 libepoxy-dev")
endif()

# Pangolin 库（libpango_*）。刻意不用 find_package(Pangolin) 导出的导入目标：
# pango_opengl 的 INTERFACE_LINK_LIBRARIES 引用了 OpenGL::EGL，该 imported
# target 在部分 CMake 版本里未定义，会在 generate 阶段直接报错。
# 这里直接按全路径链接 .so（与 libORB_SLAM3.so 的 NEEDED 完全一致）。
set(PANGOLIN_LIB_DIR "/usr/local/lib" CACHE PATH "Pangolin lib dir")
set(PANGOLIN_LIBS "")
foreach(_p
    pango_opengl pango_glgeometry pango_display pango_core pango_image
    pango_vars pango_plot pango_scene pango_tools pango_video
    pango_windowing pango_packetstream pango_geometry)
  if(EXISTS "${PANGOLIN_LIB_DIR}/lib${_p}.so")
    list(APPEND PANGOLIN_LIBS "${PANGOLIN_LIB_DIR}/lib${_p}.so")
  endif()
endforeach()
list(LENGTH PANGOLIN_LIBS _n_pango)
message(STATUS "Pangolin libs: ${_n_pango} found in ${PANGOLIN_LIB_DIR}")

# 直接链接 .so 全路径：避免 target_link_directories + -l 的顺序/同名库歧义
target_link_libraries(stereo_slam_node
  ${ORB_LIB_DIR}/libORB_SLAM3.so
  ${ORB_LIB_DIR}/libDBoW2.so
  ${ORB_G2O_DIR}/libg2o.so
  ${PANGOLIN_LIBS}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${EPOXY_LIBRARY}
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# 说明：这个发行版（ament_cmake 2.8.x）已把 ament_target_dependencies 拆成
# 独立包且不再由 find_package(ament_cmake) 提供，因此统一走现代 CMake 目标。

# 让安装后的可执行文件自带 RPATH，ros2 run 时无需手工设 LD_LIBRARY_PATH
set_target_properties(stereo_slam_node PROPERTIES
  BUILD_RPATH  "${ORB_LIB_DIR};${ORB_G2O_DIR};${PANGOLIN_LIB_DIR}"
  INSTALL_RPATH "${ORB_LIB_DIR};${ORB_G2O_DIR};${PANGOLIN_LIB_DIR}"
)

install(TARGETS stereo_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# 实机同源算法平台：双目 + IMU 的 ORB_SLAM3 IMU_STEREO + Px4Bridge。
#
# 这个目标不替代上面的 stereo_slam_node（旧 STEREO + MAVLink 路径保留作对照），
# 而是把仿真输入接到与 Nano 相同的算法/门控路径：
#   - ORB_SLAM3::System::IMU_STEREO
#   - ORB_SLAM3::px4::Bridge（vehicle_visual_odometry + 真实 health）
#   - Gazebo /clock -> epoch 时间映射，保留源时间语义
#
# 相机内参/外参来自仿真几何，见 ORB_SLAM3/config/RM27_Sim_IMU_Stereo.yaml。
# ---------------------------------------------------------------------------
set(ORB_PX4_CORE "${ORB_ROOT}/build/liborbslam3_px4_core.a")
if(NOT EXISTS "${ORB_PX4_CORE}")
  message(FATAL_ERROR
    "缺少 ${ORB_PX4_CORE}；请先在 ~/ORB_SLAM3 用 -DORB_SLAM3_WITH_PX4=ON 或 "
    "-DORB_SLAM3_BUILD_TESTS=ON 构建一次")
endif()

add_executable(stereo_inertial_sim_node
  src/stereo_inertial_sim_node.cpp
  ${ORB_ROOT}/src/Px4Bridge.cc
)

target_include_directories(stereo_inertial_sim_node PRIVATE
  ${ORB_ROOT}
  ${ORB_ROOT}/include
  ${ORB_ROOT}/include/CameraModels
  ${ORB_ROOT}/Thirdparty
  ${ORB_ROOT}/Thirdparty/Sophus
  ${ORB_ROOT}/Thirdparty/g2o
  ${ORB_ROOT}/Thirdparty/DBoW2
  ${OpenCV_INCLUDE_DIRS}
  ${PANGOLIN_INCLUDE_DIR}
)

target_compile_definitions(stereo_inertial_sim_node PRIVATE
  HAVE_EIGEN HAVE_EPOXY ORB_SLAM3_WITH_PX4=1
)

target_link_libraries(stereo_inertial_sim_node
  ${ORB_LIB_DIR}/libORB_SLAM3.so
  ${ORB_LIB_DIR}/libDBoW2.so
  ${ORB_G2O_DIR}/libg2o.so
  ${ORB_PX4_CORE}
  ${PANGOLIN_LIBS}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${EPOXY_LIBRARY}
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
  ${px4_msgs_TARGETS}
)

set_target_properties(stereo_inertial_sim_node PROPERTIES
  BUILD_RPATH  "${ORB_LIB_DIR};${ORB_G2O_DIR};${PANGOLIN_LIB_DIR}"
  INSTALL_RPATH "${ORB_LIB_DIR};${ORB_G2O_DIR};${PANGOLIN_LIB_DIR}"
)

install(TARGETS stereo_inertial_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# 真值回灌 [SIM-ONLY]：跳过 ORB-SLAM3 实时初始化，用 Gazebo 真值喂同一个
# Px4Bridge。坐标系、时间戳、Gate 判据、health 全部走实机同一份代码，
# 仿真与 Nano 的差异只剩「位姿来源」。
# 不得用于同源精度验收：health 的 tracking 质量项由真值代替。
# ---------------------------------------------------------------------------
add_executable(truth_odom_node
  src/truth_odom_node.cpp
  ${ORB_ROOT}/src/Px4Bridge.cc
)

target_include_directories(truth_odom_node PRIVATE
  ${ORB_ROOT}
  ${ORB_ROOT}/include
  ${ORB_ROOT}/Thirdparty
  ${ORB_ROOT}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(truth_odom_node PRIVATE
  HAVE_EIGEN ORB_SLAM3_WITH_PX4=1
)

target_link_libraries(truth_odom_node
  ${ORB_PX4_CORE}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${px4_msgs_TARGETS}
)

set_target_properties(truth_odom_node PROPERTIES
  BUILD_RPATH  "${ORB_LIB_DIR};${ORB_G2O_DIR}"
  INSTALL_RPATH "${ORB_LIB_DIR};${ORB_G2O_DIR}"
)

install(TARGETS truth_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
