cmake_minimum_required(VERSION 3.0.2)
project(arc380_assignment3)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
