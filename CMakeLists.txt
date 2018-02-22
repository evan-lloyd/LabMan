cmake_minimum_required(VERSION 2.6)
  PROJECT(LabMan)
 
  set(CMAKE_MODULE_PATH 
 	"${CMAKE_SOURCE_DIR}/CMake"
  )
  
  FIND_PACKAGE(OGRE)
  FIND_PACKAGE(OIS)
  
  set(BULLET_LIBRARIES
     BulletCollision
     BulletDynamics
     BulletSoftBody
     LinearMath
  )
  
  SET(BULLET_INSTALL_DIR "" CACHE PATH "Bullet installation directory")
  SET(BULLET_INCLUDE_DIR ${BULLET_INSTALL_DIR}/include)
  SET(BULLET_LIB_DIR ${BULLET_INSTALL_DIR}/lib)
 
  INCLUDE_DIRECTORIES(
 	${OGRE_INCLUDE_DIRS}
 	${OIS_INCLUDE_DIRS}
 	${BULLET_INCLUDE_DIR}
 	${PROJECT_SOURCE_DIR}/TinyXML
 	${PROJECT_SOURCE_DIR}/PhysicsSceneManager
 	${PROJECT_SOURCE_DIR}/MotionController
 	${PROJECT_SOURCE_DIR}
  )
  
  LINK_DIRECTORIES( ${BULLET_LIB_DIR} )
  
  FILE(GLOB SRCS LabMan/*.cpp)
  FILE(GLOB HDRS LabMan/*.h)
 
  ADD_EXECUTABLE(LabMan WIN32
 	${SRCS}
 	${HDRS}
  )
 
  TARGET_LINK_LIBRARIES(LabMan
 	${OIS_LIBRARIES}
 	${OGRE_LIBRARIES}
 	TinyXML
 	${BULLET_LIBRARIES}
 	PhysicsSceneManager
 	MotionController
  )
  
  ADD_CUSTOM_COMMAND(
    TARGET LabMan
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${PROJECT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/LabMan${CMAKE_EXECUTABLE_SUFFIX}
    ${PROJECT_SOURCE_DIR}/bin/LabMan_${CMAKE_CFG_INTDIR}${CMAKE_EXECUTABLE_SUFFIX}
  )
  
  FILE(GLOB SRCS TinyXML/*.cpp)
  FILE(GLOB HDRS TinyXML/*.h)
  ADD_LIBRARY(TinyXML ${SRCS} ${HDRS})
  
  FILE(GLOB SRCS PhysicsSceneManager/*.cpp)
  FILE(GLOB HDRS PhysicsSceneManager/*.h)
  ADD_LIBRARY(PhysicsSceneManager ${SRCS} ${HDRS})
  
  FILE(GLOB SRCS MotionController/*.cpp)
  FILE(GLOB HDRS MotionController/*.h)
  ADD_LIBRARY(MotionController ${SRCS} ${HDRS})
  TARGET_LINK_LIBRARIES(MotionController
    ${OGRE_LIBRARIES}
    ${BULLET_LIBRARIES}
  )
  
  ADD_DEPENDENCIES(LabMan TinyXML PhysicsSceneManager MotionController)