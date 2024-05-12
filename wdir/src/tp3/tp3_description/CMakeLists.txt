cmake_minimum_required(VERSION 3.5)
project(tp3_description)  # The name of the package

find_package(ament_cmake REQUIRED)

install(DIRECTORY gazebo launch rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
