aux_source_directory (. SRC_LIST)
include_directories (../include)

if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif()
if (NOT PCL_FOUND)
    find_package(PCL  REQUIRED)
endif ()
if (NOT EIGEN3_FOUND)
    find_package(Eigen3 REQUIRED)
endif ()
find_package(Boost  REQUIRED COMPONENTS system thread)
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

#if error
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (main ${SRC_LIST})
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
target_link_libraries(main
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)
