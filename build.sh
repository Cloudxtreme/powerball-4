#! /bin/sh
clang++ source/src/main.cpp -I /usr/local/include/irrlicht -I /usr/local/include/bullet -L/usr/local/lib -lIrrlicht -framework OpenGL -framework Cocoa -framework IOKit -o powerball $(pkg-config --libs bullet)
