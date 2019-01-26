cmake_minimum_required(VERSION 2.8.3)
project(speech_to_text)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY vocab
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/speech_to_text.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})