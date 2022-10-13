# This is the root ITK CMakeLists file.
cmake_minimum_required(VERSION 2.8.12)
 
set(PROJ sort)
project(${PROJ})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11")

# Find opencv.
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("opencv dir = ${OpenCV_INCLUDE_DIRS}")
message("Opencv lib = ${OpenCV_LIBRARIES}") 


set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/demo)

aux_source_directory(src SRC_LIST) 



add_executable(${PROJ} ${SRC_LIST}  ) 
target_link_libraries(${PROJ} ${OpenCV_LIBS})
