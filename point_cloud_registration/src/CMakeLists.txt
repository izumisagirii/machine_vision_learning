aux_source_directory (. SRC_LIST)
include_directories (../include)

find_package(OpenCV  REQUIRED)
# find_package report warning on my pc
#CMake Warning (dev) at /snap/clion/145/bin/cmake/linux/share/cmake-3.17/Modules/FindPackageHandleStandardArgs.cmake:272 (message):
#The package name passed to `find_package_handle_standard_args` (PCL_PEOPLE)
#does not match the name of the calling package (PCL).  This can lead to
#problems in calling code that expects `find_package` result variables
#(e.g., `_FOUND`) to follow a certain pattern.
# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(PCL  REQUIRED)

find_package(Eigen3 REQUIRED)
#find_package(Boost  REQUIRED COMPONENTS system thread)
include_directories(
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
#if error
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
# 3/25 I don,t know why this bug disappeared

add_executable (cloud_icp ${SRC_LIST})
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

target_link_libraries(cloud_icp
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)
