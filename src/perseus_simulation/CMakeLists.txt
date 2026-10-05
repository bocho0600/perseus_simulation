# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  perseus_simulation
  VERSION 0.0.1
  LANGUAGES CXX)

# credit https://www.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
# we always want debug info for stack tracing, so switch to RelWithDebInfo from
# Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()
# Add -Werror flag for release builds
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# INCLUDE DEPENDENCIES

# public dependencies (which should propagate)
set(THIS_PACKAGE_INCLUDE_DEPENDS)
# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

# TESTING
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(GTest REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

  enable_testing()

  add_executable(gz_bridge_config_test tests/gz_bridge_config_test.cpp)
  target_link_libraries(gz_bridge_config_test GTest::gtest_main
                        ${YAML_CPP_LIBRARIES})
  target_include_directories(gz_bridge_config_test
                             PUBLIC ${YAML_CPP_INCLUDE_DIRS})
  target_compile_options(gz_bridge_config_test PUBLIC ${YAML_CPP_CFLAGS_OTHER})

  include(GoogleTest)
  gtest_discover_tests(gz_bridge_config_test)

  ament_lint_auto_find_test_dependencies()
endif()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# GAZEBO SYSTEMS
# RegolithDig: diggable regolith bed (src/regolith_dig.cpp). Installed to lib/,
# which gazebo.launch.py puts on GZ_SIM_SYSTEM_PLUGIN_PATH.
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(gz-transport13 REQUIRED)
find_package(ZLIB REQUIRED)
add_library(RegolithDig SHARED src/regolith_dig.cpp)
target_link_libraries(
  RegolithDig PRIVATE gz-sim8::gz-sim8 gz-plugin2::register
                      gz-common5::graphics gz-transport13::gz-transport13
                      ZLIB::ZLIB)
install(TARGETS RegolithDig LIBRARY DESTINATION lib)

# INSTALL (ROS)
install(
  DIRECTORY config
            launch
            maps
            rviz
            urdf
            worlds
            models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# ROS FINALISATION
ament_package()
