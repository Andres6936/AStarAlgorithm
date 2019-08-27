AStar-Algorithm
===============

Summary
-------

This code is an efficient implementation in C++ of the A* algorithm,
designed to be used from high performance realtime applications (video games)
and with an optional fast memory allocation scheme.   

It accompanies this A* tutorial: http://www.heyes-jones.com/astar.html

The A* algorithm was described in the paper
https://ieeexplore.ieee.org/document/4082128 by Hart, Nillson and Raphael. 
Sadly Nils Nillson passed away in 2019, his work is much appreciated.

Contributions: 

* @justinhj Original code and tutorial

License
=======

This software is released under the MIT License, see License.md

Commercial Use
==============

This software has been used in AAA video games and is well tested in the wild.
Please let me know if you use this code in you games, studies or hobby projects. 

If you feel the need to pay money for this code, it is not required by the
license, but you could contribute to Unicef, a charity which helps children
worldwide,  http://www.unicef.org/ that would be awesome.

Projects using this code
========================

If you wish to be added to the list of known products/educational projects using
the code please contact me.

* Gun, Activision
* Company of Heroes and Company of Heroes Online, Relic Entertainment
* Angel Engine, a game prototyping engine http://code.google.com/p/angel-engine/
* War of Sonria, a strategy war game on PSP and Playstation 3 by Playground Squad
* Lighthouses AI contest https://github.com/marcan/lighthouses_aicontest
* Self-Driving Car Engineer Nanodegree Program https://github.com/vanAken/CarND-Path-Planning-Project

Compilation
===========

You need CMake 3.1 or above

```
cmake CMakeLists.txt
make
```

Execute the sample with `./FindPath`

Introduction
============

This implementation is intended to be simple to read yet fairly
efficient. To build it you can compile, with any recent C++ compiler,
the following files :

For path finder

* FindPath.cpp
* AStar.hpp

pathfind has no arguments. You can edit the simple map in pathfind.cpp and the start 
and goal co-ordinates to experiement with the pathfinder.

Please let me know if it doesn't work for you and I will try to help. I cannot
help if you are using an old compiler such as Turbo C++, since I update the code
to meet Ansi Standard C++ as required.

Cheers!

Justin and recently Andrés (@Andres6936)
