cmake_minimum_required(VERSION 3.5)

project(hand_on_gtsam)

message("Building hand_on_gtsam...")
message("\n")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find minimum required packages
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(yaml-cpp REQUIRED)
find_package(opengv REQUIRED)
find_package(GTSAM REQUIRED)

add_executable(2.2 hand_on_gtsam/scripts/2.2.creating_a_factor_graph.cpp)
target_link_libraries(2.2 PUBLIC
  gtsam
)

add_executable(sfm_smart_factor_exe hand_on_gtsam/scripts/SFMExample_SmartFactor.cpp)
target_link_libraries(sfm_smart_factor_exe PUBLIC
  gtsam
)
target_include_directories(sfm_smart_factor_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hand_on_gtsam/include/>
)

add_executable(calibratable_factor_exe hand_on_gtsam/scripts/SFMExample_Calibratable.cpp)
target_link_libraries(calibratable_factor_exe PUBLIC
  gtsam
)
target_include_directories(calibratable_factor_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hand_on_gtsam/include/>
)

add_executable(camera_resectioning hand_on_gtsam/scripts/CameraResectioning.cpp)
target_link_libraries(camera_resectioning PUBLIC
  gtsam
)

add_executable(selfcal_exe hand_on_gtsam/scripts/SelfCalibrationExample.cpp)
target_link_libraries(selfcal_exe PUBLIC
  gtsam
)
target_include_directories(selfcal_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hand_on_gtsam/include/>
)

add_executable(rigcal_exe hand_on_gtsam/scripts/SelfRigCalibrationExample.cpp)
target_link_libraries(rigcal_exe PUBLIC
  gtsam
)
target_include_directories(rigcal_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hand_on_gtsam/include/>
)

add_executable(rigcal_k3_exe hand_on_gtsam/scripts/SelfRigCalibrationExample_k3.cpp)
target_link_libraries(rigcal_k3_exe PUBLIC
  gtsam
)
target_include_directories(rigcal_k3_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hand_on_gtsam/include/>
)
