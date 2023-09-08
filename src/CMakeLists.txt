
project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../../)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_definitions(-D_DEBUG)
endif ()

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

AUX_SOURCE_DIRECTORY(. src)
ADD_EXECUTABLE(main ${src})