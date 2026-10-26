cmake_minimum_required(VERSION 3.10)
project(GridMapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} include/)

add_library(robot src/robot.cpp)
add_library(lidar src/lidar.cpp)
add_library(occupancygrid src/occupancy_grid.cpp)
add_library(keyboard src/keyboard.cpp)
add_library(camera src/camera.cpp)
add_library(aruco_manager src/aruco_manager.cpp)
add_library(exploration src/exploration.cpp)
add_library(behavior src/behavior.cpp)
add_library(wallfollow src/wallfollow.cpp)

# dépendances explicites entre bibliothèques (plus de dépendance à l'ordre d'édition de liens)
target_link_libraries(lidar PUBLIC ${OpenCV_LIBS})
target_link_libraries(occupancygrid PUBLIC lidar ${OpenCV_LIBS})
target_link_libraries(camera PUBLIC ${OpenCV_LIBS})
target_link_libraries(aruco_manager PUBLIC ${OpenCV_LIBS})
target_link_libraries(keyboard PUBLIC robot)
target_link_libraries(exploration PUBLIC robot lidar occupancygrid)
target_link_libraries(wallfollow PUBLIC robot lidar)

add_executable(program src/main.cpp)
target_link_libraries(program robot lidar occupancygrid keyboard camera aruco_manager
                      exploration behavior wallfollow ${OpenCV_LIBS})
