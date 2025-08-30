# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-src"
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-build"
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix"
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/tmp"
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp"
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src"
  "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/zeb/ros_workspace/build/ros-qualisys/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
