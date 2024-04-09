cmake_minimum_required(VERSION 3.5)
project(articubot_one)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(controller_manager REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(gazebo_ros REQUIRED)

#########################ROS2 Control Section
# List of ROS 2 dependencies required by this package.
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs 
)

# Find the ament_cmake package and each dependency listed above.
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Find the serialport library installed in the system.
find_library(SERIALPORT_LIB serialport)

# Create a shared library target from the specified source file.
add_library(
  ros2_control_demo_example_2
  SHARED
  hardware/diffbot_system.cpp
)

# Link the serialport library privately to the created library.
target_link_libraries(ros2_control_demo_example_2 PRIVATE ${SERIALPORT_LIB})

# Set include directories for the target. BUILD_INTERFACE is used during build,
# while INSTALL_INTERFACE is used after installation.
target_include_directories(ros2_control_demo_example_2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_2>
)
# Link against ROS 2 dependencies specified in THIS_PACKAGE_INCLUDE_DEPENDS.
ament_target_dependencies(
  ros2_control_demo_example_2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Define compile definitions for the target. Specifically, define a macro for
############################## dll export/import directives.
#target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_2_BUILDING_DLL")

# Export the plugin description file so it can be discovered by pluginlib.
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_2.xml)

############ROS2 Control Section

##### Twist Stamper
add_executable(twist_to_twist_stamped_converter src/twist_to_twist_stamped_converter.cpp)
ament_target_dependencies(twist_to_twist_stamped_converter rclcpp geometry_msgs)
install(TARGETS twist_to_twist_stamped_converter
  DESTINATION lib/${PROJECT_NAME})


################# Install
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

## ROS2 Control
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_2
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_2
)
# Install the library itself to lib folder, making it available for other projects.
install(TARGETS ros2_control_demo_example_2
  EXPORT export_ros2_control_demo_example_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# Export targets and dependencies for use by other packages.
ament_export_targets(export_ros2_control_demo_example_2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


################### Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
