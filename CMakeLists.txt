cmake_minimum_required(VERSION 2.8.3)
project(hri_safe_remote_control_system)

find_package(catkin REQUIRED
        roscpp
        std_msgs
        sensor_msgs
        greenzie_common
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
        hri_safe_remote_control_system_generate_messages_cpp
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES hri_safe_remote_control_system
        CATKIN_DEPENDS
            roscpp
            std_msgs
            sensor_msgs
            genmsg
            message_runtime
)

add_executable(safe_remote_control
        src/main.cpp
)

target_link_libraries(safe_remote_control
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS safe_remote_control
                ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark files for installation
install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

