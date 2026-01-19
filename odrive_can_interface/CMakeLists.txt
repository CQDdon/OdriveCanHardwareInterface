cmake_minimum_required(VERSION 3.10)
project(odrive_can_interface)

# ROS deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# ===== Library sources =====
set(ODRIVE_HW_SRCS
  src/can_comm.cpp
  src/odrive_motor.cpp
  src/odrive_can_system.cpp
  src/shared_memory.cpp
)

# ===== Tạo thư viện =====
add_library(${PROJECT_NAME} SHARED ${ODRIVE_HW_SRCS})

# Include dirs chuẩn ament (BUILD + INSTALL)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Link deps
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  diagnostic_msgs
  )

# Visibility (nếu cần)
target_compile_definitions(${PROJECT_NAME} PRIVATE "ODRIVE_CAN_INTERFACE_BUILDING_DLL")

# ===== Pluginlib XML =====
# đảm bảo bạn có file odrive_can_interface.xml ở gốc package
pluginlib_export_plugin_description_file(hardware_interface odrive_can_interface.xml)

# ===== CAN Test Node =====
add_executable(odrive_homing src/odrive_homing.cpp src/can_comm.cpp src/odrive_motor)
add_executable(odrive_close_loop src/odrive_close_loop.cpp src/can_comm.cpp src/odrive_motor)
add_executable(odrive_set_target src/odrive_set_target.cpp src/can_comm.cpp src/odrive_motor)
add_executable(odrive_idle src/odrive_idle.cpp src/can_comm.cpp src/odrive_motor)
add_executable(odrive_calib src/odrive_calib.cpp src/can_comm.cpp src/odrive_motor)
target_include_directories(odrive_homing    
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_homing rclcpp)
target_include_directories(odrive_close_loop    
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_close_loop rclcpp)
target_include_directories(odrive_set_target    
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_set_target rclcpp)
target_include_directories(odrive_idle    
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_idle rclcpp)
target_include_directories(odrive_calib    
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odrive_calib rclcpp)

# ===== Export headers + lib =====
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install CAN test node
install(TARGETS odrive_homing odrive_close_loop odrive_set_target odrive_idle odrive_calib
  DESTINATION lib/${PROJECT_NAME}
)

# Export target & include dirs cho package khác dùng
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle pluginlib hardware_interface)

ament_package()
