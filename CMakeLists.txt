cmake_minimum_required(VERSION 3.10)
project(r1_arm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(Threads REQUIRED)

# ----- IM6014 motor_controller (link prebuilt library) -----
if(NOT MOTOR_CONTROLLER_LIB_DIR)
  message(FATAL_ERROR "MOTOR_CONTROLLER_LIB_DIR is not set. Use build.sh to configure the platform.")
endif()

set(MOTOR_CONTROLLER_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/motor_controller/include)

find_library(MOTOR_CONTROLLER_LIB
  NAMES motor_controller
  PATHS ${MOTOR_CONTROLLER_LIB_DIR}
  NO_DEFAULT_PATH
  REQUIRED
)

add_executable(test_arm_lowlevel
  src/test_arm_lowlevel.cpp
  src/arm_motor_bus.cpp
)
# add_executable(calibrate_joints
#   src/calibrate_joints.cpp
#   src/arm_motor_bus.cpp
# )
add_executable(demo_arm_motion
  src/demo_arm_motion.cpp
  src/arm_motor_bus.cpp
)

foreach(tgt test_arm_lowlevel demo_arm_motion)
  target_include_directories(${tgt} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${MOTOR_CONTROLLER_INCLUDE_DIR}
  )
  target_link_libraries(${tgt} PRIVATE
    ${MOTOR_CONTROLLER_LIB}
    Threads::Threads
  )
endforeach()

set_target_properties(
  test_arm_lowlevel demo_arm_motion
  PROPERTIES
  BUILD_RPATH "${MOTOR_CONTROLLER_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/../thirdparty/motor_controller/lib"
)
