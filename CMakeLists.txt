cmake_minimum_required(VERSION 3.0.2)
project(panda_voice_stack)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/voice_recognizer.py
  scripts/voice_arm_control.py
  scripts/say_command.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
