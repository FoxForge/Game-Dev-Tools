# Game-Dev-Tools
My personal C++ library for game development tools

## Physics2D

After untaking a task to build a physics pipline in c# for a university module, I decided to re-write everything in c++.
It is a full working 2D physics pipline with collision handling and various utility funcitons such as raycasting and circle casting.
The pipeline is made in a very object oriented focus with a primary world class.

Existing features:
- Broad Phase & Narrow Phase Techniques
- Implementation of Dynamic AABB Tree
- Impulse collision handling
- Mid point force Integeration on rigidbodies
- Linear and Angular velocity for rigidbodies
- Utility functions such as raycasting, circle-casting, query overlaps, query points
- Object pooling system
- Allows for Trigger collisions and Physical collisions
- Collider contact callbacks (On Enter, On Stay, On Exit) for both trigger and physical

Future Improvements:
- Tagging rigidbodies
- Layering collisions (A matrix of collision masks)
- Applying collision masks to utility funcitons i.e. raycasting with a specific mask
- Attaching external arbitrary objects to the rigidbodies
- Further collider shapes including concave solutions
- Object tracking for network integrations

 * [Personal 2D Physics Blog](https://dfoxforge.wixsite.com/danielfox/research-blog/game-behaviour-building-a-2d-physics-pipeline) - Here is the blog I made for the pipeline while previously implementing in c#

The 2D physics pipeline is desgined to be a developer friendly system where it is easy to change and add anything desired.

## 2D Pathing Finding A*

A simple node-to-node 2D gridding system with developer friendly pathfinding searches. The utility of the system allows you to pass a reference to 2 dimensional
set of nodes, to become configured into a grid with neighbours where you can then perform searches and change node properties at runtime.

Existing Features:
- Provides base node class to inherit from to use the system
- Choice to setup diagonal connections (each node will either have 4 or 8 valid neighbours when configured)
- Greedy A* pathfinding
- Utility searches on paths, directly adding costs to specificed nodes when searching at runtime
- Set orientation of a node grid for 2 dimensional axes: XY, XZ, YZ

Future Improvements:
- Setup the architecture for easy heirarchical pathfinding
- Use D* lite algorithm instead of A* and/or provide options to choose
- Give utility path searches for smoother paths
- Dynamic node pooling and "occlusion" for larger searches (only load nodes where the path searches)

If all future improvements get made, it may become a task of mine to look at implementing a 3D solution. However this may also rely
on improvments made to my physics pipeline.

## Maths

For now, there are just some simple Vector classes used in the other branches.

Existing Features:
- Vector2, Vector3, Vector4
- Dot products
- Cross products
- Normalizing
- Scaling
- Vector math operations

Future Improvements:
- Matrix classes
- Perlin noise generation
- Extensions for useful mathatic operations

