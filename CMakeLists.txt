cmake_minimum_required(VERSION 3.1.0)
project(pose_seq_transform)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(posetrans STATIC PoseSeqTransformer.cc)
target_link_libraries(posetrans Eigen3::Eigen Threads::Threads)

add_executable(example example.cc)
target_link_libraries(example posetrans)