cmake_minimum_required (VERSION 2.6)

project (TOPO_GRAPH_2D)

find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

find_package(CGAL COMPONENTS Core Boost)
FIND_PACKAGE(Qt4 REQUIRED)


if ( CGAL_FOUND )

  include( ${CGAL_USE_FILE} )

  include( CGAL_CreateSingleSourceCGALProgram )

else()
  
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
  
endif()

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

  #add_subdirectory (dir)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_library (topo_graph_2d STATIC  /
  src/VoriConfig.cpp /
  src/VoriGraph.cpp /
  src/TopoGraph.cpp /
  src/RoomDect.cpp /
  src/roomGraph.cpp /
  src/Denoise.cpp /
  src/passageSearch.cpp /
  src/cgal/CgalVoronoi.cpp /
  src/cgal/AlphaShape.cpp /
  src/qt/QImageVoronoi.cpp /
  src/cgal/AlphaShapeRemoval.cpp)

include_directories (${TOPO_GRAPH_2D_SOURCE_DIR}/include)

include_directories(${CGAL_INCLUDE_DIR} ${QT_INCLUDE_DIR})

add_executable(example_segmentation test/example.cpp)
include_directories (${TOPO_GRAPH_2D_SOURCE_DIR}/include)

message(STATUS " cgal: ${CGAL_LIBRARIES} ")

TARGET_LINK_LIBRARIES(example_segmentation topo_graph_2d ${QT_LIBRARIES} ${CGAL_LIBRARIES} -lboost_unit_test_framework -lboost_filesystem -lboost_system)
