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

- Do not work on the `main` branch. 
  - This branch is meant to be used by everyone and should be a very solid base for every new student.
Do not add broken code in `main`.
  - You plan on contributing to this repo: Create your own branch from `main`.
    - Once you are done with a feature or an improvement in your personal branch/fork. You should create a pull 
request to merge your personal branch with `main`, an admin of the repo will approve your pull request.
    - Please do pull request more or less frequently. 
You should not do one giant pull request with 5000 lines of code after 2 years of work. It should be incremental.
  - If you do not plan on contributing to, nor updating, the code in `main`. You only want to use it: create your own repo and clone this one inside.
  - You plan on contributing to `main` but your research is unrelated: Create your branch from `main` to use 
and keep your research somewhere else.
    - You can also check how to use git submodule, but it may be complicated.