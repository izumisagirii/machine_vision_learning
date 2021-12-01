cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(box)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${test_union_SOURCE_DIR}/bin)
set(Darknet_INCLUDE_DIRS ${box_SOURCE_DIR}/include)
set(Kinectv3_INCLUDE_DIRS ${test_union_SOURCE_DIR}/Kinectv3)
set(Protocol_INCLUDE_DIRS ${test_union_SOURCE_DIR}/protocol)
set(Union_INCLUDE_DIRS ${test_union_SOURCE_DIR}/union_code)
if (NOT K4A_FOUND)
    find_package(K4A REQUIRED)
endif ()
if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()
if (NOT Boost_FOUND)
    find_package(Boost  REQUIRED COMPONENTS system thread)
endif ()
if (NOT PCL_FOUND)
    find_package(PCL REQUIRED)
endif ()
find_library(darknet libdarknet.so /home/hitcrt/rc/darknet)
# 注意：如果想要在自己的机器上跑代码记得将路径改为自己电脑上darknet安装的位置
# why so complicated(>_<)
# message("-----------${Kinectv3_INCLUDE_DIRS}------------")
# message("-----------${Protocol_INCLUDE_DIRS}------------")
# message("-----------${Union_INCLUDE_DIRS}------------")
include_directories(${OpenCV_INCLUDE_DIRS}
		${Darknet_INCLUDE_DIRS}
		${Kinectv3_INCLUDE_DIRS}
		${Protocol_INCLUDE_DIRS}
		${Union_INCLUDE_DIRS}
		${Boost_INCLUDE_DIRS})
# include_directories(./include)
add_executable(box test.cpp box.h box.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)
# -fPIC是动态链接库常用的编译参数，可以使动态链接库不需要根据当前程序加载到的位置重定位
# 不加-fPIC就意味着每一次动态链接库都需要重定位到当前程序所在内存位置，当代码与位置有关的时候我们这样做
# 当库不大的时候或者库需要很高的效率的时候，我们就使用-fPIC取消重定位操作，使得.so在内存中真正共享，而不用每使用一次都copy一份以重定位
target_link_libraries(box
        ${OpenCV_LIBS}
		${Boost_LIBRARIES}
        ${darknet}
		kinectv3
        union
        )
