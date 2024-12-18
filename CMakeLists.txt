cmake_minimum_required(VERSION 3.5)
project(gimbal_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(dynamixel_driver) # Install dynamixel_driver package
ament_python_install_package(${PROJECT_NAME}) # correctly install leap_ros2 package

#############
## Install ##
#############

file(GLOB SCRIPT_FILES gimbal_ros2/*.py)
install(PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

# Install all Python scripts in the launch directory (if any)
file(GLOB LAUNCH_SCRIPTS launch/*.py)
install(PROGRAMS ${LAUNCH_SCRIPTS}
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)


###################################
## ament specific configuration  ##
###################################
ament_package()


########
# test #
########

if(BUILD_TESTING)
  # Python unit tests  
  find_package(ament_cmake_pytest REQUIRED)

  set(_pytest_tests
    test/test_u2d2_connection.py
    test/test_motor_connection.py
    test/test_gimbal_registers.py
    test/test_gimbal_node_driver.py
    test/test_gimbal_connection_topics.py
    test/test_gimbal_launch_params.py
    test/test_gimbal_end_to_end.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 20
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

endif()
