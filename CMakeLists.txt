cmake_minimum_required(VERSION 2.8.3)
project(sensor_monitor)
find_package(catkin REQUIRED)
catkin_package(CATKIN_DEPENDS)

install(PROGRAMS
  nodes/velodyne_monitor/velodyne_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        PATTERN ".svn" EXCLUDE
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
)
