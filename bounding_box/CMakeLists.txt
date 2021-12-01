cmake_minimum_required(VERSION 3.12.0)
project(test_union VERSION 0.1.0)

set(EXECUTABLE_OUTPUT_PATH ${test_union_SOURCE_DIR}/bin)
set(Kinectv3_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/Kinectv3)
set(Protocol_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/protocol)
set(Union_INCLUDE_DIRS ${test_union_SOURCE_DIR}/union_code)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(K4A REQUIRED)
# if bug
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

project(test_union)

add_subdirectory(protocol)
add_subdirectory(RGBD_driver)
add_subdirectory(Kinectv3)
add_subdirectory(union_code)
add_subdirectory(track_arrow)
add_subdirectory(yolo_bounding_box)
add_subdirectory(column_identify_hsv)

include_directories(union_code
                    ${K4A_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Kinectv3_INCLUDE_DIRS}
                    ${Protocol_INCLUDE_DIRS}
                    ${Union_INCLUDE_DIRS})
add_executable(test_union main.cpp)
target_link_libraries(test_union 
                        kinectv3 protocol 
                        rgbd_camera union 
                        ${OpenCV_LIBS} 
                        ${K4A_LIBS} 
                        ${PCL_LIBRARIES} 
                        ${Boost_LIBRARIES}
                        )
