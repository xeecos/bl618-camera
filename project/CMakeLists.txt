cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

# target_sources(app PRIVATE ../src/jpeg.c)
sdk_set_main_file(../src/main.c)

project(bl616_camera)
