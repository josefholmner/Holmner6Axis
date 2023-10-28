cmake_minimum_required(VERSION 3.10)

project(Holmner6Axis LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 17)

FILE(GLOB HEADERS "include/Holmner6Axis/*.h")
FILE(GLOB SOURCES "src/*.cpp")

add_executable(Holmner6Axis ${SOURCES} ${HEADERS})

target_include_directories(Holmner6Axis PUBLIC "include")

install (TARGETS Holmner6Axis
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
