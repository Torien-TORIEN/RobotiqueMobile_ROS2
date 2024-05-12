cmake_minimum_required(VERSION 3.5)
project(my_first_package) # The name of the package

# ament_cmake is the build system for CMake based packages in ROS 2, https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # as we are doing python nodes

file(GLOB scripts src/*) # to process all the py scripts in the src directory at once (instead of having an install line for each script)
install(PROGRAMS ${scripts} DESTINATION lib/${PROJECT_NAME}) # to install the scripts

ament_package()
