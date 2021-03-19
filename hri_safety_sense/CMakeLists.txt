cmake_minimum_required(VERSION 2.8.3)
project(hri_safety_sense)

find_package(catkin REQUIRED
        roscpp
        std_msgs
        sensor_msgs
        genmsg
        message_runtime
        message_generation
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_service_files(
        DIRECTORY srv
        FILES
        EmergencyStop.srv
        KeyString.srv
        KeyValue.srv
)

add_message_files(
        DIRECTORY msg
        FILES
        SrcDisplay.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

add_library(${PROJECT_NAME}
        src/JoystickHandler.cpp
        src/SerialInterface.c
        src/VehicleInterface.c
        src/VscProcess.cpp
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}

)
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES hri_safety_sense
        CATKIN_DEPENDS
            roscpp
            std_msgs
            sensor_msgs
            genmsg
            message_runtime
)

add_executable(safe_remote_control src/main.cpp)
target_link_libraries(safe_remote_control
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

 
