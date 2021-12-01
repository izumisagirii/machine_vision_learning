CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(LIVOX)
set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Debug)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -pg -march=native")


if (NOT PCL_FOUND)
    find_package(PCL REQUIRED)
endif ()
if (NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS system thread)
endif ()
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_subdirectory(livox_lidar)

ADD_EXECUTABLE(test main.cc)
TARGET_LINK_LIBRARIES(test
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        livox_lidar
        )
#ADD_EXECUTABLE(test_thread test_thread.cc)
#
#TARGET_LINK_LIBRARIES(test_thread
#        ${PCL_LIBRARIES}
#        ${Boost_LIBRARIES}
#        livox_lidar
#        )
