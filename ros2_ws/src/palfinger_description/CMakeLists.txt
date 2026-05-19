cmake_minimum_required(VERSION 3.8)
project(palfinger_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  models
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
