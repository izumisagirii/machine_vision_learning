cmake_minimum_required(VERSION 3.0.0)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
# -march=native表示的是利用当前编译器进行代码优化
# -Wall表示让GCC显示尽可能多的错误信息
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_STANDARD 14)
project(columns_identify VERSION 0.1.0)
add_subdirectory(src)
add_subdirectory(livox_lidar)
add_subdirectory(hik-driver)
add_subdirectory(protocol)
