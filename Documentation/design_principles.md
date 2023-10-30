# Design and Code Principles

## Important points

- Minimize dependencies.
- Make it work on any brand new linux machine.
- Make it so a new student can launch it easily.
- Make your nodes independent and safe (one node crashing should make another node crash).
- Do not delete, break or deeply change behavior of existing nodes that are working inside of main. 
Create a new function in an existing node, or a new node or new package instead.
- Important parameters should be ros2 parameter set inside launch_settings.py and imported in the launch file. 
A node's code should not change when parameters -- such as leg dimensions, motor speed -- are changed.

## For your research

### Do not work on the `main` branch

- This branch is meant to be used by everyone and should be a very solid base for every new student to do research.
Do not add broken code in `main`.
- You plan on contributing to this repo: Create your own branch from `main`.
  - Once you are done with a feature or an improvement in your personal branch/fork. You should create a pull 
request to merge your personal branch with `main`, an admin of the repo will approve your pull request.
  - Please do pull request more or less frequently. 
You should not do one giant pull request with 5000 lines of code after 2 years of work. It should be incremental.
- If you do not plan on contributing to, nor updating, the code in `main`. You only want to use it: create your own repo and clone this one inside.
- You plan on contributing to `main` but your research is unrelated: Create your branch from `main` to use 
and keep your research somewhere else in another repo.
  - You can also check how to use git submodule, but it may be complicated.

### Contributing and general ros2 advice

- Adding a new tool should be done by creating a new package.
  - Example: You create a package to move based on keys pressed on the keyboard. With two nodes: 
  One node handles the keyboard, another handles the movement (sending messages to other nodes in already existing packages). 
- Adding a new object (a thing containing functions and states) should be done by creating a new node.
  - Example: You add a node to support a joystick.
- Adding a new functionality should be done by adding a new ros2 component (topic, service, action ...) to an existing node.
  - Example: You add a subscription in the movement node to react to joystick messages.
- Improving a functionality should be done by updating an existing function's code. Preferabily without modifying any ros2 component.
  - Example: You disable keyboard control if the joystick is used.
- You should understand the structure of this repo's ros2 in order to modify it.
  Take time to decide of where and what to modify.

## Overall structure and design to respect 

The structure is meant to be like a pyramid with several levels: 
- Higher level means the abstraction and responsibilities are higher.
- Levels can be composed of several nodes.
  - One node per motor gives 12 motor nodes
- Levels should ONLY be dependent on nodes below:
  - If levels below are missing: current level waits or changes behavior (but does not crash).
  - If levels above are missing: current level DOES work.
  - If node on same level is missing: current level DOES work.
- Commands flow down
- Information flows up

The basic nodal structure of this repo:
```  
level XX       /    \      
              /  ..  \     
level 05     /  Step  \     x1
level 04    /   Leg    \    x4
level 03   /     IK     \   x4
level 02  /     Joint    \  x12
level 01 /      Motor     \ x12
```

### In English with example

Role of each node:
- Step: Synchronizes the movement of several legs
- Leg: Moves the tip of a leg from one target to another in a controlled motion
- IK: Finds the joints angles for the leg tip to be on target
- Joint: Interprets and correct the joint angle (applies angle limits, flips angle, zeros, send to the corresponding motor)
- Motor: Receives angle and rotates.

The joint handle everything related to the joint. It receives angles from the level above: IK (Inverse Kinematics), 
and send commands to the motor node. It also sends the current angle of the joint to the levels above (IK and more).

The joint nodes shouldn't rely on data of levels above, so the joint does not have access to the position of the leg tip
or the step direction. The joint nodes only cares about level below, so only about the motor. Similarly, the joint node
cannot send commands to any node above, so a joint node cannot ask directly for a step to be done.

Levels should ONLY be dependent on nodes below. So if the node of leg #1 is not available:
- The IK, Joint and motors are still working fine (because level below)
- The other legs also work (because same level)
- The step node waits or needs to change its behavior by changing the gait (because level above)

