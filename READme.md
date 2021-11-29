# 21S_CS380 - Computer Graphics Project(Undergraduate - http://vclab.kaist.ac.kr/cs380/) based on OpenGL

## HW2 - 3D Coordinate Systems

Your program will use OpenGL to draw two cubes as well as some scenery, the cubes will become robots in the future. You will implement a number of different viewpoints, and allow the cubes to be moved around the scene. They will move with respect to various frames, including the two cubes' frames and external points of view.

## HW3 - Quaternions and Arcball

In this project you will implement the code necessary to apply rigid body transformations, represented as a translation (coordinate vector) and a rotation (quaternion). You will also implement the Arcball interface for rotations.

## HW4 Graphics - Robots and Part Picking

Implementi a system for drawing articulated bodies, as well as the ability to select objects on the screen. Your program will draw two robots, instead of two cubes as done in the previous assignments, and allow the manipulation of robot parts in a way the preserves the hierarchical structure of the robot. You will allow the user to rotate and translate the robots as well each of their movable joints.

## HW5 - Key Frame Animator(Part 1)

In this project you will complete the code necessary for a keyframe animation system with linear interpolation. In such a system, the user defines the state of the world at fixed key times. Intermediate frames are generated via interpolation (for now just linear) to yield an animation.

## HW6 - Material System

In this assignment we will also upgrade the scene graph infrastructure to allow for multiple types of materials with diferent GLSL shaders in a single scene. One particular material that you will help implement is a bump mapping material.

## HW7 - Key Frame Animator(Part 2)

In this assignment you will replace in your linear interpolation code with Catmull-Rom interpolation. During the animation, you will create intermediate states by evaluating the Bezier curves at intermediate times, and then use these intermediate states to display an intermediate frame. For the orientations you will need to use quaternion splines.

## HW8 - Meshes and Subdivision

The purpose of this assignment is to use a mesh data structure and implement Catmull-Clark subdivision.