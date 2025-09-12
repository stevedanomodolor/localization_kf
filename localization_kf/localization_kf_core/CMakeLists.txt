cmake_minimum_required(VERSION 3.8)
project(localization_kf_core)


add_compile_options(-g)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)


include_directories(include)
include_directories(${rclcpp_components_INCLUDE_DIRS})

set(executable_name localization_core)
set(library_name ${executable_name}_node)


set (dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_util
  rclcpp_components
  gps_msgs
  sensor_msgs
  ros_gz_interfaces
  nav_msgs
  geometry_msgs
  Eigen3
  )


# localization library 
# navigator library
add_library(${library_name} SHARED
  src/localization_core.cpp
 src/ekf.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

# navigator executable
add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
   ${GeographicLib_LIBRARIES}
)

rclcpp_components_register_nodes(${library_name} "localization_kf_core::LocalizationCore")

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  )
# install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
  
install(TARGETS ${library_name} ${executable_name}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${library_name})
ament_package()