# 3D-Mesh-LOD-Tool
Class project from SFU Geometric Modelling course

•	Utilized winged edge data structure and C++/OpenGL to load, save and display 3D obj mesh files

•	Implemented Loop and Butterfly subdivision schemes to apply surface smoothing and generate higher detailed models

•	Implemented quadric-based edge decimation with multiple-choice algorithm to generate lower detailed models while preserving the original structure

Requires OpenGL and nanogui

Download and add nanogui to project_folder/lib/nanogui (see CMakeLists.txt)

cmake

make

./mcaq

Based on the following two papers: 
Surface Simplification Using Quadric Error Metrics https://www.cs.cmu.edu/~./garland/Papers/quadrics.pdf
Fast Mesh Decimation by Multiple-Choice Techniques https://www.graphics.rwth-aachen.de/media/papers/mcd_vmv021.pdf
