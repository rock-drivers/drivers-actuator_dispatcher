#PROJECT(interfaces)
#cmake_minimum_required(VERSION 2.6)
#INCLUDE_DIRECTORIES("tasks")
#SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
#INCLUDE(interfacesBase)


cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(actuator_dispatcher 0.1)
rock_standard_layout()
# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

