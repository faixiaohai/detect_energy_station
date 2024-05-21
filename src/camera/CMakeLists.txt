cmake_minimum_required(VERSION 3.17)
project(camera)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()



set(CMAKE_PREFIX_PATH /opt/ros/iron/share/cv_bridge/cmake)
find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(interfaces REQUIRED)


ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${interfaces_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC camera_data/include)

target_include_directories(${PROJECT_NAME} PUBLIC camera_data/include include)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC camera_data/lib/64)
  install(
    DIRECTORY camera_data/lib/64
    DESTINATION lib)
endif()



target_link_libraries(${PROJECT_NAME} 
    -lX11
    ${OpenCV_LIBS}
    ${interfaces_LIBRARIES})
    
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
    INSTALL_TO_SHARE
    config
    launch
)

# ament_package()