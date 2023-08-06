cmake_minimum_required(VERSION 3.16)
project(arduino_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  arduino_interface
  SHARED
  src/arduino_interface.cpp
)
target_compile_features(arduino_interface PUBLIC cxx_std_17)
target_include_directories(arduino_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/arduino_interface>
)
ament_target_dependencies(
  arduino_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ARDUINO_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arduino_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/arduino_interface
)

install(TARGETS arduino_interface
  EXPORT export_arduino_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# This is terrible, but it makes libserial work
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)
target_include_directories(arduino_interface PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(arduino_interface PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})

## EXPORTS
ament_export_targets(export_arduino_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()