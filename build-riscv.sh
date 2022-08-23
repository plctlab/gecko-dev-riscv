#!/usr/bin/bash

export LDFLAGS=-L/lvm/1/code/github/zlib-1.2.11.dfsg
export CXXFLAGS=-I/lvm/1/code/github/zlib-1.2.11.dfsg
export CFLAGS=-I/lvm/1/code/github/zlib-1.2.11.dfsg

export CC=/usr/bin/riscv64-linux-gnu-gcc
export CXX=/usr/bin/riscv64-linux-gnu-g++

./mach build
