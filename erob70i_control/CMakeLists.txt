cmake_minimum_required(VERSION 3.8)
project(erob70i_control)

set(CMAKE_BUILD_TYPE Release)
find_package(ament_cmake REQUIRED)

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

include_directories(
  ${ETHERLAB_DIR}/include
)

add_executable(
  erob70i_tightening
  src/erob70i_tightening.cpp)

add_executable(
  erob70i_releasing
  src/erob70i_releasing.cpp)

target_include_directories(
  erob70i_tightening
  PRIVATE
  ${ETHERLAB_DIR}/include
)

target_include_directories(
  erob70i_releasing
  PRIVATE
  ${ETHERLAB_DIR}/include
)

target_link_libraries(erob70i_tightening ${ETHERCAT_LIB})
target_link_libraries(erob70i_releasing ${ETHERCAT_LIB})

install(TARGETS erob70i_tightening erob70i_releasing
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
