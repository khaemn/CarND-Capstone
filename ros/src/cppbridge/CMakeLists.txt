cmake_minimum_required(VERSION 2.8.3)
project(cppbridge)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

catkin_package()

add_executable(talker src/talker.cpp)

target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)

target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(bridge src/bridge.cpp)

target_link_libraries(bridge ${catkin_LIBRARIES} z ssl uv uWS)

