# rrt-mp

 This work addresses one of the most challenging problems in robotics namely kinodynamic motion planning. Motion planning plays an important role in autonomous navigation, searches for a collision-free path from an initial state to a final state. To tackle the problem a C++ library that implements an extension of the optimal Rapidly-exploring Random Trees(RRT*) with a database of pre-computed motion primitives has been created. This approach alleviates the computational load and allows for motion planning in dynamic or partially known environments. The library utilizes hash-tables to look up the motion primitives in an efficient manner and adopts the infrastructure provided by the Open Motion Planning Library (OMPL). 
 
 The main dependencies of this project is Boost and OMPL. The easiest way to install OMPL though ROS on Ubuntu or other any Linux based system. 
``` bash
sudo apt-get install ros-`rosversion -d`-ompl
```
 Installing the boost
```bash
sudo apt-get install libboost-all-dev
```

Then to build the project, go to the directory 
```bash
mkdir build && cd build/
cmake .. 
make
```

Than the test codes can be run in the build folder as 
```bash
./anyTestExectuable
```
