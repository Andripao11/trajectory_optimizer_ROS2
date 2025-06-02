cmake_minimum_required(VERSION 3.5)
project(trajectory_optimizer)

# Trova le dipendenze ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(slam_msgs REQUIRED)
#find_package(path_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Trova Eigen
find_package(Eigen3 REQUIRED)

# Configura CasADi (C++)
find_path(CASADI_INCLUDE_DIR
  NAMES casadi.hpp
  HINTS /home/andrea/casadi/casadi
)

if(CASADI_INCLUDE_DIR)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
  include_directories(${CASADI_INCLUDE_DIR})
else()
  message(FATAL_ERROR "CasADi headers not found at ${CASADI_INCLUDE_DIR}")
endif()

find_library(CASADI_LIBRARY
    NAMES casadi
    PATHS ${HOME}/casadi/build/lib/
)

if(CASADI_LIBRARY)
    message(STATUS "Found CasADi lib: ${CASADI_LIBRARY}")
    link_directories(${CASADI_LIBRARY})
else()
    message(FATAL_ERROR "CasADi library not found at ${CASADI_LIBRARY}")
endif()

# Includi directory
include_directories(
  include
  ${CASADI_INCLUDE_DIR}
)

# Linka librerie
link_directories(
  /home/andrea/casadi/build/lib
)

# Eseguibile 1
add_executable(tro_node
  src/main.cpp
  src/utils/TRO.cpp
)
ament_target_dependencies(tro_node
  rclcpp std_msgs nav_msgs visualization_msgs
) #slam_msgs path_interfaces 
target_link_libraries(tro_node ${CASADI_LIBRARY})

# Eseguibile 2
add_executable(optimizer_node
  src/opt.cpp
  src/utils/SGSmooth.cpp
  src/utils/optimizer.cpp
)
ament_target_dependencies(optimizer_node
  rclcpp std_msgs 
) #slam_msgs path_interfaces nav_msgs
target_link_libraries(optimizer_node ${CASADI_LIBRARY})

# Eseguibile 3
add_executable(midline_node
  src/midline.cpp
  src/utils/GRO.cpp   # o qualunque file sorgente per GRO
  # altri file se servono
)
ament_target_dependencies(midline_node
  rclcpp
#  std_msgs
#  nav_msgs
#  slam_msgs
#  path_interfaces
)
target_link_libraries(midline_node ${CASADI_LIBRARY})


#############
## Install ##
#############

# Eseguibili
install(TARGETS
  tro_node
  optimizer_node
  midline_node
  DESTINATION lib/${PROJECT_NAME}
)

# Cartella di launch (obbligatorio per far trovare i .launch.py)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# (opzionale ma consigliato) parametri, config, ecc.
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

#File CSV, mappe, ecc.
#install(
#  DIRECTORY data
#  DESTINATION share/${PROJECT_NAME}/
#)

# Se in futuro aggiungi altre cartelle (scripts, cfg, ecc.) ripeti lo schema:
# install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}/)

ament_package()
