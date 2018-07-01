# Human Body Reconstruction
This project is an implementation of 2002 Research Paper [SCAPE : Shape Completion and Animation of People](http://ai.stanford.edu/~drago/Projects/scape/scape.html)


## Pre-requisite tools
1. OpenGL
2. GLU
3. FreeGLUT
4. ASSIMP
5. LibPCA
6. Ceres-Solver v1.12 (New version is slower)
7. cvxpy v0.4.11 (New version is slower and requires some changes to code)


## Compilation and Execution
* cd human_body_reconstruction
* mkdir cmake-build-dir
* cd cmake-build-dir
* cmake ..
* make

### Learn Body Model
* ./learn \<Body-Model Directory\>
* OR
* python3 HBR_learn.py <Body-Model Directory\>

### Generate Body Model
* ./ generate \<Body-Model Directory\> \<Point cloud PTCLD file\>
* OR
* python3 HBR_complete.py \<Body-Model Directory\> \<Point cloud PTCLD file\> \<Actual Mesh PLY file\> \<Output Mesh file\>



## Notes:
* If CVXPY version newer than v0.4.11 is installed, some changes to code is required.
    * `cvx.Variable(3,3)` is chaged to `cvx.Variable(shape=(3,3))`