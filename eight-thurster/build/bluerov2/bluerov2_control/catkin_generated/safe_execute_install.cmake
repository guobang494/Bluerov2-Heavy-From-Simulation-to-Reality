execute_process(COMMAND "/home/zeb/eight-thurster/build/bluerov2/bluerov2_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zeb/eight-thurster/build/bluerov2/bluerov2_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
