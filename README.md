# 3D robot modelling 

## :rocket: Roadmap {#roadmap}

- [x] Pkgs for robot modelling
- [ ] [Generalitites about Robot modelling using URDF](#robmod)
- [ ] Create a pkg for the robot description
- [ ] Generalities about xacro files
- [ ] Creating a robot description for a seven DOF robot manipulator
- [ ] Working with the joint_state_publisher and robot_state_publisher for
- [ ] Creating the robot description for a differential wheeled robot
- [ ] Creating the robot description for the [explab_2nd][1]
- [ ] Using the ROS MoveIt! and navigation stack for the assignment 
  
## Main packages for robot modelling {#pkgs}

ROS uses a standard metapackage for designing and creating models named `robot_model`.

 It basically consists in a set of packages whose purpose is to provide support for the 3D model description reflecting the same features of the real hardware. Those packages are:

1. `urdf` - it contains a C++ parser for the Unified Robot Description Format which stands for an **XML** file representing a general robot model 
2. `kdl_parser` - It stands for Kinematic and Dynamics Library (KDL). It contains parsers tools needed for creating a **kinematic tree**. This is used to:
   1. publish the joint states
   2. forward and inverse the kinematic of the robot 
3. `joint_state_publisher` - It contains a node responsible for:
   1. reading the robot model description
   2. finding its joints
   3. publishing joints' values by means of sliders
4. `robot_state_publisher` - It reads the current robot  joint states and publishes the 3D poses of each robot link using the kinematics tree 
5. `collada_urdf`

## About urdf 
 As mentioned in the [previoius section](#pkgs) the URDF parsers are critical for parsing a robot model with some sensors and a working environment 

 Only robot with links arranged in **tree structure** can be described through URDF. Hence, the robot must have:

 - *rigid* links (flexible ones are not allowed)
 - each link is connected with others by means of *joints* 

The URDF basically consists in a bunch of XML tags, and it represents the kinematic and dynamic description of the robot, its visual represetntation and its collision model  

### about Xacro

XML Macros (aka Xacro) show some add-ons to improve readbility and for building complex robot's descriptions 

> :warning: xacro files should always be converted in URDF for being employable

## Generalities about robot modelling using URDF {#robmod}

### checking the model

For cheking the model:

1. save the current model with `.urdf` extension
2. check if the current model contains errors (by parsing the urdf tag and show the potentially occuring error)

   ```sh
   check_urdf nameOfTheModel.urdf
   ```

3. for viewing the structure of the robot links and joints graphically (two files will be generated with both `.gv` and `.pdf` extensions)

    ```sh
    urdf_to_graphiz nameOfTheModel.urdf
    ```

4. For inspecting the model:
   
    ```sh
    evince nameOfTheModel.pdf
    ```
### using Rviz 

For taking a look at the robot structure by means of RVIZ, please run  

```sh
roslaunch pkgName demo.launch
```





## References 
[1]: https://www.github.com/fedehub/explab_2nd
[2]:  moveit.com
