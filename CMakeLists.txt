cmake_minimum_required(VERSION 3.25)

project(Mimzy LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include(cmake/clang_format.cmake)
include(cmake/libraries.cmake)

FetchContent_MakeAvailable(Yoth)

if(${PROJECT_IS_TOP_LEVEL})

add_clang_format_target(format "build")

set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(examples)

endif()

add_subdirectory(mimzy)
