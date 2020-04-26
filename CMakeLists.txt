project(FUCKED)
cmake_minimum_required(VERSION 3.15) #idk why this one

set(CMAKE_CXX_STANDARD 20)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_INCLUDE_DIR})

include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories( ${PROJECT_SOURCE_DIR}/include )

add_executable(${PROJECT_NAME} main.cpp include/utils.h src/mapReuse.cpp include/mapReuse.h)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR})