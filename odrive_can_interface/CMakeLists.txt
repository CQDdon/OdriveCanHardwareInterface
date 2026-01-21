cmake_minimum_required(VERSION 3.10)
project(odrive_can_interface)

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# =====================
# Library sources
# =====================
set(ODRIVE_HW_SRCS
  src/can_comm.cpp
  src/odrive_motor.cpp
  src/odrive_can_system.cpp
  src/shared_memory.cpp
)

# =====================
# Shared library (PLUGIN)
# =====================
add_library(${PROJECT_NAME} SHARED
  ${ODRIVE_HW_SRCS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  diagnostic_msgs
)

# Needed for Windows, harmless on Linux
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "ODRIVE_CAN_INTERFACE_BUILDING_DLL"
)

# =====================
# Pluginlib export
# =====================
pluginlib_export_plugin_description_file(
  hardware_interface
  odrive_can_interface.xml
)

# =====================
# Test / utility nodes
# =====================
set(ODRIVE_TEST_NODES
  odrive_homing
  odrive_close_loop
  odrive_idle
  odrive_calib
)

foreach(node ${ODRIVE_TEST_NODES})
  add_executable(${node}
    src/${node}.cpp
    src/can_comm.cpp
    src/odrive_motor.cpp
  )

  target_include_directories(${node}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${node}
    rclcpp
  )
endforeach()

# HWI test node (link plugin lib)
# add_executable(odrive_hwi_test src/odrive_hwi_test.cpp)
# target_link_libraries(odrive_hwi_test ${PROJECT_NAME})
# ament_target_dependencies(odrive_hwi_test
#   rclcpp
#   rclcpp_lifecycle
#   hardware_interface
# )

# Thread test
# add_executable(thread_test src/thread_test.cpp)
# target_link_libraries(thread_test ${PROJECT_NAME})
# ament_target_dependencies(thread_test rclcpp)

# =====================
# Install
# =====================
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES odrive_can_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ${ODRIVE_TEST_NODES}
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Export for downstream packages
# =====================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
)

ament_package()
