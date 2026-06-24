#!/bin/bash
# Script untuk kompilasi dan menjalankan aplikasi sekaligus

echo "Compiling C++ engine..."
if g++ -std=c++17 -O2 main.cpp -o rrt_engine; then
    echo "Compilation successful."
else
    echo "Compilation failed!"
    exit 1
fi

echo "Launching GUI..."
python3 gui_visualizer.py