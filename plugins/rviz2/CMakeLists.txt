cmake_minimum_required(VERSION 3.5)
project(atos_rviz_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(atos_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(atos_rviz_plugins_headers_to_moc
  include/object_monitor_display.hpp
)
foreach(header "${atos_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(atos_rviz_plugins_moc_files "${header}")
endforeach()

set(atos_rviz_plugins_source_files
  src/object_monitor_display.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${atos_rviz_plugins_source_files}
  ${atos_rviz_plugins_moc_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
	${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
	rviz_ogre_vendor::OgreMain
	rviz_ogre_vendor::OgreOverlay
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
	rclcpp
	atos_interfaces
	rviz_common
	rviz_default_plugins
	geometry_msgs
	pluginlib
	rviz_rendering
	tf2
	tf2_geometry_msgs
	tf2_ros
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin_description.xml DESTINATION "share/${PROJECT_NAME}")

ament_package()
