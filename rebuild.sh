#!/bin/bash
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make -j$(nproc)
cd ..
echo "Done!"
