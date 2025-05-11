set(MIMZY_SOURCES
  ${PROJECT_SOURCE_DIR}/mimzy/accelerators/bvh.cpp
  ${PROJECT_SOURCE_DIR}/mimzy/mathematics/bounding_box.cpp
  ${PROJECT_SOURCE_DIR}/mimzy/mathematics/triangle.cpp
)

add_library(${PROJECT_NAME} STATIC
  ${MIMZY_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Yoth
)