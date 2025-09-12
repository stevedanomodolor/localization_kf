cmake_minimum_required(VERSION 3.8)
project(localization_kf_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(localization_kf_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
    include/localization_kf_rviz_plugin/navigation_panel.hpp
    )

add_library(localization_kf_rviz_plugin src/navigation_panel.cpp ${MOC_FILES})
target_include_directories(localization_kf_rviz_plugin
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(localization_kf_rviz_plugin
    rclcpp
    rviz_common
    std_srvs
    rclcpp_action
    pluginlib
    localization_kf_msgs
    std_msgs
)

install (TARGETS localization_kf_rviz_plugin
    EXPORT export_localization_kf_rviz_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins_description.xml
        DESTINATION share/${PROJECT_NAME}/
)

install(FILES icons/classes/NavigationPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)

ament_export_include_directories(include)
ament_export_targets(export_localization_kf_rviz_plugin)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()
