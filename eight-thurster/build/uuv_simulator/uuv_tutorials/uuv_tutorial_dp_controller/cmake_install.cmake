# Install script for directory: /home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zeb/test-8/eight-thurster/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zeb/test-8/eight-thurster/build/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller/catkin_generated/installspace/uuv_tutorial_dp_controller.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_tutorial_dp_controller/cmake" TYPE FILE FILES
    "/home/zeb/test-8/eight-thurster/build/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller/catkin_generated/installspace/uuv_tutorial_dp_controllerConfig.cmake"
    "/home/zeb/test-8/eight-thurster/build/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller/catkin_generated/installspace/uuv_tutorial_dp_controllerConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_tutorial_dp_controller" TYPE FILE FILES "/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uuv_tutorial_dp_controller" TYPE PROGRAM FILES "/home/zeb/test-8/eight-thurster/build/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller/catkin_generated/installspace/tutorial_dp_controller.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uuv_tutorial_dp_controller" TYPE DIRECTORY FILES "/home/zeb/test-8/eight-thurster/src/uuv_simulator/uuv_tutorials/uuv_tutorial_dp_controller/launch" REGEX "/[^/]*\\~$" EXCLUDE)
endif()

