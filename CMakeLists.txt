cmake_minimum_required(VERSION 3.10)
project(gtsam_examples)

set(CMAKE_CXX_STANDARD 17)

# Find GTSAM
find_package(GTSAM REQUIRED)

# Set the source directory
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Collect all .cpp files from src/
file(GLOB SOURCES "${SRC_DIR}/*.cpp")

# Loop over each .cpp file to create an executable
foreach(SOURCE_FILE ${SOURCES})
    get_filename_component(EXE_NAME ${SOURCE_FILE} NAME_WE)
    add_executable(${EXE_NAME} ${SOURCE_FILE})
    target_include_directories(${EXE_NAME} PRIVATE ${SRC_DIR} ${GTSAM_INCLUDE_DIR})
    target_link_libraries(${EXE_NAME} gtsam tbb)
endforeach()
