cmake_minimum_required(VERSION 2.8)
project(Kalman CXX)

#mfc
set(BUILD_TESTING 0)

# TODO: Use VERSION argument to project command after raising the minimum
# cmake version to 3.0
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

include(CTest)

# mfc: needed for csv reader
find_package( Threads REQUIRED )
message(${CMAKE_THREAD_LIBS_INIT}) #pthread library found?

set(KALMAN_CXX_FLAGS "-std=c++11")

option(KALMAN_BUILD_EXAMPLES "Build the Kalman examples" ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Eigen3 REQUIRED)

set(HEADERS
    # Base
    include/kalman/KalmanFilterBase.hpp
    include/kalman/StandardBase.hpp
    include/kalman/SquareRootBase.hpp
    include/kalman/StandardFilterBase.hpp
    include/kalman/SquareRootFilterBase.hpp

    # EKF
    include/kalman/ExtendedKalmanFilter.hpp
    include/kalman/SquareRootExtendedKalmanFilter.hpp

    # UKF
    include/kalman/UnscentedKalmanFilterBase.hpp
    include/kalman/UnscentedKalmanFilter.hpp
    include/kalman/SquareRootUnscentedKalmanFilter.hpp

    # Models
    include/kalman/MeasurementModel.hpp
    include/kalman/SystemModel.hpp
    include/kalman/LinearizedMeasurementModel.hpp
    include/kalman/LinearizedSystemModel.hpp

    # Utils
    include/kalman/Matrix.hpp
    include/kalman/Types.hpp
)

include_directories(
    "include"
    ${Eigen3_INCLUDE_DIR}
)

# Flags
set(CMAKE_CXX_FLAGS "${KALMAN_CXX_FLAGS} -Wall -pedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DEIGEN_NO_DEBUG")

# Examples
if(KALMAN_BUILD_EXAMPLES)
  add_executable(example_robot1 "examples/Robot1/main.cpp")
  add_executable(example_steps "examples/Steps/main.cpp")
  add_executable(force_steps "examples/ForceSteps/main.cpp")
  add_executable(filter_csv_data "examples/Steps/filter_csv_data.cpp")
  
  # mfc: needed for csv reader
  target_link_libraries(filter_csv_data ${CMAKE_THREAD_LIBS_INIT})

endif()

# Unit Tests
set(TESTS
    # EKF
    test/ExtendedKalmanFilter.cpp
    
    # UKF
    test/UnscentedKalmanFilterBase.cpp
    test/UnscentedKalmanFilter.cpp
    test/SquareRootUnscentedKalmanFilter.cpp
    
    # Utils
    test/StandardBase.cpp
    test/SquareRootBase.cpp
    test/Matrix.cpp
)


#
# Install/Uninstall support
#
include(CMakePackageConfigHelpers)
set(KALMAN_INCLUDE_DIR include/
  CACHE PATH "The directory where Kalman headers are installed")
set(KALMAN_CONFIG_CMAKE_PATH lib${LIB_SUFFIX}/cmake/kalman
  CACHE PATH "The directory where the CMake files are installed")

set(KALMAN_USE_FILE ${KALMAN_CONFIG_CMAKE_PATH}/UseKalman.cmake)

configure_package_config_file(cmake/KalmanConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/KalmanConfig.cmake
  INSTALL_DESTINATION ${KALMAN_CONFIG_CMAKE_PATH}
  PATH_VARS KALMAN_INCLUDE_DIR KALMAN_USE_FILE)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/KalmanConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

# Install targets
install(DIRECTORY include/kalman DESTINATION include)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/UseKalman.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/KalmanConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/KalmanConfigVersion.cmake
  DESTINATION ${KALMAN_CONFIG_CMAKE_PATH})

# Add uninstall target
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/KalmanUninstall.cmake)
