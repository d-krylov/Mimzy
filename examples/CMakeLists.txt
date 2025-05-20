FetchContent_MakeAvailable(tinyobjloader)

set(EXECUTABLES
  example
)

set(EXAMPLES_TOOLS
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/tools.cpp
)

find_package(SDL3 REQUIRED CONFIG REQUIRED COMPONENTS SDL3-shared)

foreach(exe ${EXECUTABLES})
  add_executable(${exe} ${EXAMPLES_TOOLS} ${CMAKE_CURRENT_SOURCE_DIR}/${exe}/${exe}.cpp)
  target_link_libraries(${exe} PRIVATE Mimzy tinyobjloader SDL3::SDL3)
  target_include_directories(${exe} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()