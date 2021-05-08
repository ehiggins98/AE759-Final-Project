project(ae759_final_project)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Debug)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
        ${PROJECT_SOURCE_DIR}
        include
        ${GAZEBO_INCLUDE_DIRS}
        )
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ArduCopterIRLockPlugin SHARED src/ArduCopterIRLockPlugin.cpp)
target_link_libraries(ArduCopterIRLockPlugin ${GAZEBO_LIBRARIES})

add_library(FinalProjectPlugin SHARED src/final_project_plugin/FinalProjectPlugin.cpp)
add_library(Motors SHARED src/final_project_plugin/Motors.cpp)
add_library(SimWirelessReceiver SHARED src/final_project_plugin/SimWirelessReceiver.cpp)
add_library(EKF SHARED src/final_project_plugin/EKF.cpp)
add_library(ArduPilot SHARED src/final_project_plugin/ArduPilotInterface.cpp)
add_library(Logger SHARED src/final_project_plugin/Logger.cpp)
add_library(Camera SHARED src/final_project_plugin/Camera.cpp)

target_link_libraries(FinalProjectPlugin ${GAZEBO_LIBRARIES})
target_link_libraries(FinalProjectPlugin Motors)
target_link_libraries(FinalProjectPlugin SimWirelessReceiver)
target_link_libraries(FinalProjectPlugin EKF)
target_link_libraries(FinalProjectPlugin ArduPilot)
target_link_libraries(FinalProjectPlugin Logger)
target_link_libraries(FinalProjectPlugin Camera)
target_link_libraries(Motors ${GAZEBO_LIBRARIES})
target_link_libraries(SimWirelessReceiver ${GAZEBO_LIBRARIES})
target_link_libraries(EKF Eigen3::Eigen)
target_link_libraries(FinalProjectPlugin Eigen3::Eigen)
target_link_libraries(ArduPilot ${GAZEBO_LIBRARIES})
target_link_libraries(Camera ${OpenCV_LIBS})

install(TARGETS ArduCopterIRLockPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
install(TARGETS FinalProjectPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})

install(DIRECTORY models DESTINATION ${GAZEBO_MODEL_PATH}/..)
