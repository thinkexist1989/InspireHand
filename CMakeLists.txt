cmake_minimum_required(VERSION 3.10)

project(InspireHand)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(test main.cpp)
target_link_libraries(test Boost::system)