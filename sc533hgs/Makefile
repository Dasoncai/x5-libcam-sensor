-include ./version.mk

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

TARGET = $(notdir $(shell pwd))

ifneq (${version},)
upv = $(TARGET)
endif

include ../Makefile
