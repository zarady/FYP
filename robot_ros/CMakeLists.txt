cmake_minimum_required(VERSION 2.8.3)
project(robot_ros)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

# Install directories (config, launch, meshes, urdf)
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# === ADD THIS SECTION TO INSTALL PYTHON SCRIPTS ===
catkin_install_python(PROGRAMS
  scripts/omni_drive_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
