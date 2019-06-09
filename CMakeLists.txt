cmake_minimum_required(VERSION 2.6)
project(plane_detection)

find_package (OpenCV REQUIRED)

include_directories(include)
add_executable(
    plane_detection
    include/Octree.h
    include/Plane.h
    include/Ply.h
    include/Point.h
    include/PointCloud.h
    include/Ransac.h
    include/RGB.h
    include/UnionFind.h
    include/Vec3.h
    src/Octree.cpp
    src/Plane.cpp
    src/Ply.cpp
    src/PointCloud.cpp
    src/Ransac.cpp
    src/main.cpp
)
    
target_link_libraries(plane_detection ${OpenCV_LIBS})
