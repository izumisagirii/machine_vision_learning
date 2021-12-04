set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
    cmake_policy(SET CMP0074 OLD)
endif(COMMAND cmake_policy)
#set(ZED_DIR "/usr/local/zed_previous")
find_package(ZED 2 REQUIRED)
if (NOT CUDA_FOUND)
    find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
endif ()
if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()
if (NOT PCL_FOUND)
    find_package(PCL  REQUIRED)
endif ()
if (NOT Boost_FOUND)
    find_package(Boost  REQUIRED COMPONENTS system thread)
endif ()
file(GLOB SRC_LIST *.cpp)
ADD_LIBRARY(zedcamera SHARED ${SRC_LIST})
TARGET_LINK_LIBRARIES(zedcamera ${ZED_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${CUDA_npps_LIBRARY}
        ${CUDA_NPP_LIBRARIES_ZED}
        ${Boost_LIBRARIES}
        )
target_include_directories(zedcamera PUBLIC ${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ./)
