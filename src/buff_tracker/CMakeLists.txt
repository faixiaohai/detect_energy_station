file(GLOB_RECURSE src CONFIGURE_DEPENDS include/* src/*.cpp)
project(tracker)

add_library(${PROJECT_NAME} ${src})

target_include_directories(${PROJECT_NAME} PUBLIC ../buff_detect/include)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS}) 
