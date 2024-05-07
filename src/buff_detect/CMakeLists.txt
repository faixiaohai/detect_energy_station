file(GLOB_RECURSE source ./src/*.cpp)

add_library(mylib ${source})

target_include_directories(mylib PUBLIC include)
target_include_directories(mylib PUBLIC ${OpenCV_INCLUDE_DIRS})
