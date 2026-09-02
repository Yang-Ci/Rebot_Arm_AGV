cmake_minimum_required(VERSION 3.8)
project(rebotarm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/generate_lab_map.py scripts/generate_rs_agv_sdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
