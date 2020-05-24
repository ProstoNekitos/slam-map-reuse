project(slam_map_reuse)
cmake_minimum_required(VERSION 3.15) #idk why this one

set(CMAKE_CXX_STANDARD 20)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories( ${PROJECT_SOURCE_DIR}/include )

add_executable(${PROJECT_NAME} main.cpp include/utils.h src/mapReuse.cpp include/mapReuse.h include/Params.h)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(ICP icp.cpp include/utils.h src/mapReuse.cpp include/mapReuse.h include/Params.h)
target_link_libraries(ICP ${PCL_LIBRARIES})

add_executable(BUFF buffer.cpp include/utils.h src/mapReuse.cpp include/mapReuse.h include/Params.h)
target_link_libraries(BUFF ${PCL_LIBRARIES})

add_executable(massif_to_txt data/descriptors/data_to_txt.cpp)
