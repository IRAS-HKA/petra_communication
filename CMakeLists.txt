cmake_minimum_required(VERSION 3.5)
project(petra_communication)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aip_interfaces REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES 
    "rclcpp"
    "aip_interfaces"
)

add_executable(Communication src/Communication.cpp)
ament_target_dependencies(Communication ${DEPENDENCIES})

add_executable(Screen src/Screen.cpp)
ament_target_dependencies(Screen ${DEPENDENCIES})

add_executable(Keyboard src/Keyboard.cpp)
ament_target_dependencies(Keyboard ${DEPENDENCIES})

install(TARGETS 
  Communication
  Screen
  Keyboard
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
