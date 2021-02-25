cmake_minimum_required(VERSION 3.10)

project(InspireHand)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(test main.cpp InspireHand.cpp)
target_link_libraries(test Boost::system)