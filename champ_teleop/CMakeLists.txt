cmake_minimum_required(VERSION 3.5)
project(champ_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   my_champ_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

