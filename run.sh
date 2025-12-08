#!/bin/bash

mkdir -p build
cd build

if [ ! -f build.ninja ]; then
    echo "🔧 Configuring project with Ninja..."
    
    if [ -f Makefile ]; then
        echo "   (Detected old Makefile, cleaning build cache...)"
        rm -rf *
    fi
    
    cmake -G "Ninja" ..
fi

echo "🔨 Building with Ninja..."
ninja

if [ $? -eq 0 ]; then
    echo "✅ Build successful. Starting..."
    echo "--------------------------------------------------"
    
    cd ..
    
    ./build/ip9_task_scheduling "$@"
else
    echo "❌ Build failed!"
    exit 1
fi
