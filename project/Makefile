BL616_CAMERA_PATH ?= ../
BL_SDK_BASE ?= $(BL616_CAMERA_PATH)/../bouffalo_sdk

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl618cam
CROSS_COMPILE ?= riscv64-unknown-elf-
NOW_REAL_PATH:= $(realpath ./)
BOARD_DIR = ${NOW_REAL_PATH}
# add custom cmake definition
#cmake_definition+=-Dxxx=sss

include $(BL_SDK_BASE)/project.build
