cmake_minimum_required(VERSION 3.8)
project(riptide_echosounder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# External libraries driver
include(FetchContent) # once in the project to include the module
FetchContent_Declare(sea_scan_echo_driver GIT_REPOSITORY https://github.com/Teusner/Sea_Scan_Echo_Marine_Sonic_Driver.git GIT_TAG main)
FetchContent_MakeAvailable(sea_scan_echo_driver)

FetchContent_Declare(rtac_asio GIT_REPOSITORY https://github.com/pnarvor/rtac_asio.git GIT_TAG master)
FetchContent_MakeAvailable(rtac_asio)

FetchContent_Declare(NemaTode GIT_REPOSITORY https://github.com/ckgt/NemaTode.git GIT_TAG master)
FetchContent_MakeAvailable(NemaTode)

include_directories(include)

# Compiling riptide echosounder library
add_library(riptide_echosounder src/riptide_echosounder.cpp)
target_link_libraries(riptide_echosounder rtac_asio sea_scan_echo_driver NemaTode)
ament_target_dependencies(riptide_echosounder rclcpp sensor_msgs)

# Compiling riptide echosounder driver
add_executable(riptide_echosounder_driver src/riptide_echosounder_driver.cpp)
target_link_libraries(riptide_echosounder_driver PRIVATE riptide_echosounder)

install(TARGETS
  riptide_echosounder_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
