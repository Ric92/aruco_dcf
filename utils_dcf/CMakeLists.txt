INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()


add_executable(aruco_dcf aruco_dcf.cpp)
add_executable(aruco_dcf_mm aruco_dcf_markermap.cpp)

target_link_libraries(aruco_dcf		aruco opencv_calib3d opencv_highgui ${THREADLIB})
target_link_libraries(aruco_dcf_mm		aruco opencv_calib3d opencv_highgui ${THREADLIB})

if(${USE_ROS})
  find_package(catkin REQUIRED roscpp geometry_msgs sensor_msgs nav_msgs image_transport cv_bridge)
  if(${catkin_FOUND})
    catkin_package(
      CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs image_transport cv_bridge
    )
    
    target_compile_definitions(aruco_dcf PUBLIC USING_ROS)
    target_include_directories(aruco_dcf PUBLIC ${catkin_INCLUDE_DIRS}) #${cv_bridge_INCLUDE_DIRS})
    target_link_libraries(aruco_dcf ${catkin_LIBRARIES})    #${cv_bridge_LIBRARIES})
    add_dependencies(aruco_dcf ${catkin_EXPORTED_TARGETS})
  endif()
endif()

INSTALL(TARGETS aruco_dcf aruco_dcf_mm RUNTIME DESTINATION bin)
