cmake_minimum_required(VERSION 3.5)
project(tp3_nodes)  # The name of the package

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


# si on a des nodes
file(GLOB scripts src/*) # to process all the py scripts in the src directory at once (instead of having an install line for each script)
install(PROGRAMS ${scripts} DESTINATION lib/${PROJECT_NAME}) # to install the scripts

ament_package()