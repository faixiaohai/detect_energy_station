file(GLOB_RECURSE source ./src/*.cpp)
add_library(mylib ${source})
target_include_directories(mylib PUBLIC ../detect/include)
find_package(OpenCV REQUIRED)
target_link_libraries(mylib ${OpenCV_LIBS})