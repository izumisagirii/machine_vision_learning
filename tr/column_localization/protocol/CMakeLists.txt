project(protocol)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
if (NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS system thread filesystem)
endif ()
file(GLOB SOURCE_LIST_PROTOCOL *.cpp)
add_library(protocol
        SHARED
        ${SOURCE_LIST_PROTOCOL}
        )
target_include_directories(protocol PUBLIC ${Boost_INCLUDE_DIRS} ./)
target_link_libraries(protocol ${Boost_LIBRARIES})
target_sources(protocol PRIVATE ${SRC_LIST})
