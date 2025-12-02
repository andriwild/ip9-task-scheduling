#!/bin/bash
cd build
make -j$(nproc) 
#clear
./ip9_task_scheduling "$@"

