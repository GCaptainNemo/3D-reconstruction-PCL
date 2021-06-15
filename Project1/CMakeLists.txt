cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")

PROJECT("surface_reconstruction")

# opencv + PCL + Python
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(PythonLibs REQUIRED)


include_directories(
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS}
)

file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)


add_executable(
${PROJECT_NAME} 
${DIR_SRCS}
${DIR_HEADERS}
)

target_link_libraries(${PROJECT_NAME} 
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
${PYTHON_LIBRARIES}
)



