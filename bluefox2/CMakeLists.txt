cmake_minimum_required(VERSION 3.8)
project(bluefox2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

set(THIS_PACKAGE_INCLUDE_DEPENDS
	ament_cmake
  sensor_msgs
  mvIMPACT
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include/bluefox2)
include_directories(${mvIMPACT_INCLUDE_DIRS})

set(HEADER_FILES include/bluefox2/bluefox2.hpp include/bluefox2/bluefox2_setting.hpp)

add_library(bluefox2
    src/bluefox2.cpp
    src/bluefox2_setting.cpp
    ${HEADER_FILES}
    )

ament_target_dependencies(bluefox2
    sensor_msgs
    mvIMPACT
    )

ament_export_targets(bluefox2Targets HAS_LIBRARY_TARGET)

target_include_directories(bluefox2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

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
install(TARGETS bluefox2
    EXPORT bluefox2Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

install(DIRECTORY include/bluefox2
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  )

ament_package()