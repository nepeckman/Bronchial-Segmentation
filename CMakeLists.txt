cmake_minimum_required(VERSION 3.7)
Project(BronchialSegmentation)

FIND_PACKAGE(ITK)
IF(ITK_FOUND)
	INCLUDE(${ITK_USE_FILE})
ENDIF(ITK_FOUND)

FIND_PACKAGE(VTK)
IF(VTK_FOUND)
	INCLUDE(${VTK_USE_FILE})
ENDIF(VTK_FOUND)

INCLUDE_DIRECTORIES(${BronchialSegmentation_SOURCE_DIR})
ADD_EXECUTABLE(BronchialSegmentation BronchialSegmentation.cxx)

TARGET_LINK_LIBRARIES(BronchialSegmentation
	${VTK_LIBRARIES} ${ITK_LIBRARIES}
	)

