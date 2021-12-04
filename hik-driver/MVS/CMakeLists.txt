cmake_minimum_required(VERSION 3.5)
project(MVS)
#add_library(mvs )
find_package(OpenCV REQUIRED)
find_package(Boost  REQUIRED COMPONENTS system thread)
set(CMAKE_CXX_STANDARD 14)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/module1)
link_directories(/opt/MVS/lib/aarch64)
else()
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/module)
link_directories(/opt/MVS/lib/64)
endif()

include_directories(/opt/MVS/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS}
        include/)
include_directories(/opt/MVS/include)
#add_library($(MVCAM_COMMON_RUNENV))
find_package(MVSCAM REQUIRED)
add_executable(MVS main.cpp MVS.cpp include/MVS.h)

target_link_libraries(MVS
        ${MVSCAM_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES})

add_executable(MVS_test grab.cpp MVS.cpp include/MVS.h)

target_link_libraries(MVS_test
        ${MVSCAM_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES})
