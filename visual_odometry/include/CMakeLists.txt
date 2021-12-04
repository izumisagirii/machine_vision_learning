if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()
if (NOT Boost_FOUND)
    find_package(Boost  REQUIRED COMPONENTS system thread)
endif ()
if (NOT Eigen3_FOUND)
    find_package(Eigen3 REQUIRED)
endif ()
if (NOT Sophus_FOUND)
    find_package(Sophus REQUIRED)
endif ()
add_library(odometry SHARED odometry.h odometry.cpp)
target_include_directories(odometry PUBLIC ${OpenCV_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} 
                            ${Sophus_INCLUDE_DIRS}
                            )
target_link_libraries(odometry ${OpenCV_LIBRARIES}
                               ${Boost_LIBRARIES}
                               )