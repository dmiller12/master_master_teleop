cmake_minimum_required(VERSION 3.8)
project(master_master)

set(CMAKE_CXX_STANDARD 98)
set(CMAKE_C_STANDARD 99)

# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(Barrett REQUIRED)

## libbarrett
include_directories("${BARRETT_INCLUDE_DIRS}")
link_directories("${BARRETT_LIBRARY_DIRS}")
add_definitions(${BARRETT_DEFINITIONS})

add_executable(master_master ${CMAKE_SOURCE_DIR}/src/master_master.cpp)
target_link_libraries(master_master ${BARRETT_LIBRARIES})
target_include_directories(master_master PRIVATE ${CMAKE_SOURCE_DIR}/include)
install(TARGETS master_master DESTINATION bin) # test install with staging set

