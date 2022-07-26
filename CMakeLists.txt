cmake_minimum_required(VERSION 3.17)
project(test5)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.8 REQUIRED  )
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
file(GLOB SRC_LIST "src/*.cpp" "src/*.c"  )
message(STATUS "SRC_LIST=${SRC_LIST}")

add_executable (test5 main.cpp ${SRC_LIST})
target_link_libraries (test5 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (test52 main2.cpp ${SRC_LIST})
target_link_libraries (test52 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
