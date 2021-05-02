project(ae759_final_project)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gazebo REQUIRED)
include_directories(
        ${PROJECT_SOURCE_DIR}
        include
        ${GAZEBO_INCLUDE_DIRS}
        )
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ArduCopterIRLockPlugin SHARED src/ArduCopterIRLockPlugin.cpp)
target_link_libraries(ArduCopterIRLockPlugin ${GAZEBO_LIBRARIES})

add_library(FinalProjectPlugin SHARED src/final_project_plugin/FinalProjectPlugin.cpp)
add_library(Motors SHARED src/final_project_plugin/Motors.cpp)
add_library(SimWirelessReceiver SHARED src/final_project_plugin/SimWirelessReceiver.cpp)
target_link_libraries(FinalProjectPlugin ${GAZEBO_LIBRARIES})
target_link_libraries(FinalProjectPlugin Motors)
target_link_libraries(FinalProjectPlugin SimWirelessReceiver)
target_link_libraries(Motors ${GAZEBO_LIBRARIES})
target_link_libraries(SimWirelessReceiver ${GAZEBO_LIBRARIES})
target_link_libraries(FinalProjectPlugin Eigen3::Eigen)

install(TARGETS ArduCopterIRLockPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
install(TARGETS FinalProjectPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})

install(DIRECTORY models DESTINATION ${GAZEBO_MODEL_PATH}/..)
