cmake_minimum_required(VERSION 3.5)
project(prius_description)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_envs.sh.in")
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
