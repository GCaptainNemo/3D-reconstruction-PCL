cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")

PROJECT("surface_reconstruction")

# opencv + PCL + Python
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(PythonLibs REQUIRED)


MESSAGE(STATUS "Project: SERVER") #打印相关消息消息


include_directories(
${PROJECT_SOURCE_DIR}/include
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
)

set(SRC 
${PROJECT_SOURCE_DIR} src/kitti_io_obj.cpp
${PROJECT_SOURCE_DIR} src/lvx_io_obj.cpp 
${PROJECT_SOURCE_DIR} src/main.cpp
${PROJECT_SOURCE_DIR} src/pc_operator.cpp 
${PROJECT_SOURCE_DIR} src/utils.cpp
${PROJECT_SOURCE_DIR} src/texturing.cpp

)

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} 
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${PYTHON_LIBRARIES}
)



