cmake_minimum_required(VERSION 2.8.12)
project(car_path)

add_subdirectory(pybind11)
pybind11_add_module(car_path car_path.cpp)