cmake_minimum_required(VERSION 2.6)
project(QRotate)

AUX_SOURCE_DIRECTORY(. SRC_LIST)
add_executable(QRotate ${SRC_LIST})
target_link_libraries(QRotate)
