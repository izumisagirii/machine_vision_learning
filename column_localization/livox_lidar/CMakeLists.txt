if (NOT PCL_FOUND)
    find_package(PCL  REQUIRED)
endif ()
if (NOT LIVOXLIDAR_FOUND)
    find_package(LIVOXLIDAR  REQUIRED)
endif()
if (NOT Boost_FOUND)
    find_package(Boost  REQUIRED COMPONENTS system thread)
endif ()
if (NOT OPENMP_FOUND)
    find_package(OpenMP)
endif ()
if (OPENMP_FOUND)
    message(STATUS "Found openmp")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
add_library(livox_lidar SHARED
        livox.cc
        )
target_include_directories(livox_lidar PUBLIC  ./ ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${LIVOXLIDAR_INCLUDE_DIRS} )
target_link_libraries(livox_lidar  ${Boost_LIBRARIES} ${LIVOXLIDAR_LIBS} ${PCL_LIBRARIES} )

