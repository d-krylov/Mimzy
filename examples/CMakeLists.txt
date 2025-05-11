FetchContent_MakeAvailable(tinyobjloader)

set(EXECUTABLES
  example
)

find_package(SDL3 REQUIRED CONFIG REQUIRED COMPONENTS SDL3-shared)

foreach(exe ${EXECUTABLES})
  add_executable(${exe} ${CMAKE_CURRENT_SOURCE_DIR}/${exe}/${exe}.cpp)
  target_link_libraries(${exe} PRIVATE Mimzy tinyobjloader SDL3::SDL3)
endforeach()