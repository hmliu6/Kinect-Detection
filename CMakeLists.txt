project(kinect-test)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

add_executable(${PROJECT_NAME} ${SRC_LIST})

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)

# Find freenect, to set necessary variables
find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)

# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

# Include directories to get freenect headers
include_directories($ENV{HOME}/freenect2/include)

# Link freenect libraries with the project
target_link_libraries(${PROJECT_NAME} ${freenect2_LIBRARIES} ${OpenCV_LIBS})
