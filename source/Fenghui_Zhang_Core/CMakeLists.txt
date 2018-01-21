add_library(FenghuiZhangCore INTERFACE)

target_include_directories(FenghuiZhangCore
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/inc
    )

target_sources(FenghuiZhangCore
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/edge.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/face.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/id_generator.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/identifiable.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/object_store.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/renderable_object.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/topology_object.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vertex.cc
    )
