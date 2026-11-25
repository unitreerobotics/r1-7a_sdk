cmake_minimum_required(VERSION 3.13)
project(r1_arm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

option(BUILD_WITH_ALGO "Build the Pinocchio-based algorithm examples" OFF)

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

if(BUILD_WITH_ALGO)
  find_package(Eigen3 REQUIRED)

  if(NOT ARM_TARGET_PLATFORM)
    set(ARM_TARGET_PLATFORM "${CMAKE_SYSTEM_PROCESSOR}")
  endif()
  if(ARM_TARGET_PLATFORM MATCHES "^(arm64|aarch64)$")
    set(ARM_TARGET_PLATFORM aarch64)
  elseif(ARM_TARGET_PLATFORM MATCHES "^(x86|x86_64|amd64)$")
    set(ARM_TARGET_PLATFORM x86_64)
  else()
    message(FATAL_ERROR "Unsupported ARM_TARGET_PLATFORM: ${ARM_TARGET_PLATFORM}")
  endif()

  set(PINOCCHIO_ROOT "${CMAKE_SOURCE_DIR}/algo/pinocchio")
  set(PINOCCHIO_LIB_DIR "${PINOCCHIO_ROOT}/lib/${ARM_TARGET_PLATFORM}")
  foreach(lib pinocchio_default pinocchio_parsers)
    set(lib_path "${PINOCCHIO_LIB_DIR}/lib${lib}.so")
    if(NOT EXISTS "${lib_path}")
      message(FATAL_ERROR "Missing Pinocchio library: ${lib_path}")
    endif()
    add_library(${lib} SHARED IMPORTED)
    set_target_properties(${lib} PROPERTIES IMPORTED_LOCATION "${lib_path}")
  endforeach()

  foreach(tgt gravity_compensation roboplan_record)
    add_executable(${tgt} src/${tgt}.cpp src/arm_motor_bus.cpp)
    target_include_directories(${tgt} PRIVATE
      ${CMAKE_SOURCE_DIR}/include
      ${MOTOR_CONTROLLER_INCLUDE_DIR}
      ${PINOCCHIO_ROOT}/include
    )
    # Match the Boost variant layout used by the prebuilt Pinocchio libraries.
    target_compile_definitions(${tgt} PRIVATE
      BOOST_MPL_LIMIT_LIST_SIZE=30
      BOOST_MPL_LIMIT_VECTOR_SIZE=30
      BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
      BOOST_FUSION_INVOKE_MAX_ARITY=12
    )
    # Keep Pinocchio libraries in the executable's direct dependencies.
    target_link_options(${tgt} PRIVATE "-Wl,--no-as-needed" "-Wl,--disable-new-dtags")
    target_link_libraries(${tgt} PRIVATE
      ${MOTOR_CONTROLLER_LIB}
      pinocchio_default
      pinocchio_parsers
      Eigen3::Eigen
      Threads::Threads
    )
    set_target_properties(${tgt} PROPERTIES
      BUILD_RPATH "${MOTOR_CONTROLLER_LIB_DIR};${PINOCCHIO_LIB_DIR}"
    )
  endforeach()
endif()
