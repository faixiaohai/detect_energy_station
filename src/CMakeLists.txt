cmake_minimum_required(VERSION 3.17)
project(buff)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(buff_detect)
add_subdirectory(buff_tracker)
add_subdirectory(camera)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC buff_detect/include)
target_include_directories(${PROJECT_NAME} PUBLIC buff_tracker/include)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
                                        Eigen3::Eigen
                                        mylib
                                        tracker
                                        camera)

