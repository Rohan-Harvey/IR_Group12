cmake_minimum_required(VERSION 3.8)
project(parking_lot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config docs scripts models urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS run.sh run_autonomous.sh DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/teleop_robot.py DESTINATION lib/${PROJECT_NAME} RENAME teleop_robot)

ament_package()
