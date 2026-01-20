cmake_minimum_required(VERSION 3.8)
project(bluefox2_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
	ament_cmake
  bluefox2
  sensor_msgs
  image_transport
  camera_info_manager
  diagnostic_updater
)

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(bluefox2_ros
    src/bluefox2_ros.cpp
    )
target_include_directories(bluefox2_ros PUBLIC
                           include
    )
ament_target_dependencies(bluefox2_ros
    bluefox2
    sensor_msgs
    image_transport
    camera_info_manager
    diagnostic_updater
    )

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

# Installs
install(TARGETS bluefox2_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

install(TARGETS bluefox2_ros
    DESTINATION lib/bluefox2_ros)

install(DIRECTORY include
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  )

install(DIRECTORY launch config
  DESTINATION share/bluefox2_ros
  )

ament_package()
