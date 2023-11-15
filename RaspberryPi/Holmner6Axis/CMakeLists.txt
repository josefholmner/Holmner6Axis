cmake_minimum_required(VERSION 3.10)

project(Holmner6Axis LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-psabi")

FILE(GLOB HEADERS "include/Holmner6Axis/*.h")
FILE(GLOB SOURCES "src/*.cpp")

add_executable(Holmner6Axis ${SOURCES} ${HEADERS})

target_link_libraries(Holmner6Axis PRIVATE pigpio)

install (TARGETS Holmner6Axis
        RUNTIME DESTINATION bin
)
