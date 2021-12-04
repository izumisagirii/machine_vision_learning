if (NOT OPENCV_FOUND)
    find_package(OpenCV REQUIRED)
endif ()
if (NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS system thread)
endif ()
set(CMAKE_CXX_STANDARD 14)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/modules/module1)
    link_directories(/opt/MVS/lib/aarch64)
else ()
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/modules/module)
    link_directories(/opt/MVS/lib/64)
endif ()

find_package(MVSCAM REQUIRED)
file(GLOB SRC_LIST *.cpp)
add_library(MVS SHARED "")
target_include_directories(MVS PUBLIC ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ./ /opt/MVS/include)
target_link_libraries(MVS
        ${MVSCAM_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        )
target_sources(MVS PRIVATE ${SRC_LIST})
