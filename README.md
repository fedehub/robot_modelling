<img align="left" width="80" height="80" src="https://github.com/fedehub.png" alt="github icon">
<h1>3D robot modelling</h1>

A brief overview about urdf files, xacro files etc aimed at exploring the robot description. 

## :rocket: Roadmap

- [x] Pkgs for robot modelling
- [x] [Generalitites about Robot modelling using URDF](#robmod)
- [x] Create a pkg for the robot description
- [x] Generalities about xacro files
- [ ] Creating a robot description for a seven DOF robot manipulator
- [ ] Working with the joint_state_publisher and robot_state_publisher for
- [ ] Creating the robot description for a differential wheeled robot
- [ ] Creating the robot description for the [explab_2nd][1]
  - [ ] Using the ROS MoveIt! and navigation stack for the assignment 

## Main packages for robot modelling

ROS uses a standard **metapackage** for designing and creating models named `robot_model`.

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

<img width="844" alt="Screenshot 2022-06-25 at 16 44 50" src="https://user-images.githubusercontent.com/61761835/175778596-a07abb4d-38c9-4b2a-9092-8e849e0dfb1f.png">

Only robot with links arranged in **tree structure** can be described through URDF. Hence, the robot must have:

 - *rigid* links (flexible ones are not allowed)
 - each link is connected with others by means of *joints* 

The URDF basically consists in a bunch of XML tags, and it represents the kinematic and dynamic description of the robot, its visual represetntation and its collision model  

<img width="844" alt="Screenshot 2022-06-25 at 16 46 32" src="https://user-images.githubusercontent.com/61761835/175778674-19eed518-2340-43ec-b425-72a17bfff162.png">


### about Xacro

XML Macros (aka Xacro) show some add-ons **to improve readbility** and for building complex robot's descriptions. The Xacro allows to create macros inside the robot description quite useful.

<img width="847" alt="Screenshot 2022-06-25 at 16 48 25" src="https://user-images.githubusercontent.com/61761835/175778741-9115c323-4923-4df5-9878-fd46134491bb.png">

> :warning: xacro files should always be converted in URDF for being employable

### using xacros

> use this extension for xacro files, as the first declaration of the file ("name" stands for the name of the robot)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="…">
```

we can define properties, as for constants (to avoid hardcoding), i.e.:

```xml
<xacro:property name="cluedo_link_length" value="0.4" />
<xacro:property name="cluedo_link_radius" value="0.3" />
```
> So that in the urdf file, we can simply refer to the link's lenght through its name, i.e. in the following …

```xml
<cylinder length="${cluedo_link_length}" radius="${cluedo_link_radius}"/>
```

we can also define math expressions using basic operations, i.e.

```xml
<cylinder length="${cluedo_link_length+2}" radius="${cluedo_link_radius}"/>
<cylinder length="${cluedo_link_length/3}" radius="${cluedo_link_radius}"/>
<cylinder length="${cluedo_link_length-0.4}" radius="${cluedo_link_radius}"/>
```

or we can define macros for describing an i.e. an _inertial matrix_, where the only value that actually changes is its **mass**, taken as parameter

```xml
<xacro:macro name="my_inertial_matrix" params="mass"> 
    <inertial>
        <mass value="${mass}" />
        <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.5" />
    </inertial>
</xacro:macro>
```
### 🔀 From Xacro to urdf 

To manually convert the xacro file into urdf, please run

```sh
  rosrun xacro xacro simple_robot.xacro --inorder > simple_robot_generated.urdf
```

Otherwise, to include such conversion inside a **launch file**, add these two lines:

```sh

<param name="robot_description" command="$(find xacro)/xacro --inorder $(find robot_modelling)/urdf/simple_robot.xacro"/>

```
Being `robot_modelling` the name of the package



## Generalities about robot modelling using URDF 

### checking the model: step by step

<p align="center">
  <img width="509" alt="Screenshot 2022-06-25 at 16 58 08" src="https://user-images.githubusercontent.com/61761835/175779158-35e95979-9db9-44f5-8d03-c6e3eb95b6a4.png">
</p>

For cheking the model:

1. save the current model with `.urdf` extension
2. check if the current model contains errors (by parsing the urdf tag and show the potentially occuring error)

   ```sh
   check_urdf simple_robot.urdf
   ```

3. for viewing the structure of the robot links and joints graphically (two files will be generated with both `.gv` and `.pdf` extensions)

    ```sh
    urdf_to_graphiz simple_robot.urdf
    ```

4. For inspecting the model:
   
    ```sh
    evince simple_robot.pdf
    ```
    
### checking the model: in a wink …

For checking the model, and launch the simulation:

1. enable "execution" permission

    ```sh
    chmod +x test.py
    ```    
  
3. run the script

    ```sh
    ./test.py 
    ```


### using Rviz 

For taking a look at the robot structure by means of RVIZ, please run  

```sh
roslaunch pkgName demo.launch
```

## Tags

For further information about tags [click here][3] for the official documentation reference 

↪️ Also, remember to substitute values whenever `…` occurs!

robot def:

```urdf
<?xml version="1.0"?>
<robot name="<name of the robot>" 
  <link> … </link>
  <link> … </link>
  
  <joint> … </joint>
  <joint> … </joint> 
</robot>
```
gazebo plugins inclusion:

```urdf
<gazebo reference="link_ex"> 
  <material>Gazebo/Black</material>
</gazebo>
```

Link definition 
> Please note that **collision** and **inertia** parameters are needed, for allowing Gazebo to properly simulate the robot model itsef 

```urdf
<link name="<name of the link>"> 
<inertial> 
<mass value ="…">
<inertia ixx="…" ixy="…" ixz="…" iyy="…" iyz="…" izz="…"/>
</inertial> 
  <visual> 
    <geometry>
    < … length="…" radius="…"/> 
    </geometry>
    <origin rpy="… … …" xyz="… … …"/>
  </visual>
  <collision>
    <geometry>
    <cylinder length="…" radius="…"/> 
    </geometry>
    <origin rpy="…" xyz="…"/>
  </collision>
 </link>
 ```
Please note the choice of the link's geometry is up to you!

<details>
<summary>  About the geometry tag ... ⬇️ </summary>
There are three shapes of geometries to choose between:
 
- The **box** shape

  ```
  <geometry>
	  <box size="0.4 0.2 0.08" />
  </geometry>

  ```

- The **cylinder** shape

  ```
  <geometry>
	  <cylinder length="0.05" radius="0.2" />
  </geometry>

  ```
  
  
- The **sphere** shape
    
  ```
  <geometry>
	  <sphere radius="0.08" />
  </geometry>

  ```

  
</details>

 
 
Joint definition
> Please note that the effort is intended as _the maximum force_ supported by the joint; Moreover, we refer to revolute type joint with **radians**  and to prysmatic type with **meters**

```urdf
<joint name="<name of the joint>"> 
  <parent link="…"/>
  <child link="…"/> 
  <origin xyz="… … …"/>
  <axis xyz="… … …" />
  <calibration … />
  <dynamics damping … />
  <limit effort … /> 
</joint>
 ```

# MoveIt configuration

# Some remarks

Please note that any implementation of the moveit configuration has been made through a specific version (1.15) of the framework itself. This is the same employed for the **Experimental Robotics Laboratory course** and it is employed within the package [explab_2nd][1]

⚠️ if you have used `apt-get ...` for downloading [MoveIt][4], please run the script `moveit_fixer.py`, where all the instructions needed for obtaining the 1.15 moveit framework, have been provided (thanks to [Professor Recchiuto](https://github.com/CarmineD8) )

```sh

chmod +x moveit_fixer.py
./moveit_fixer.py

```

## References 

[1]: https://www.github.com/fedehub/explab_2nd
[2]: https://moveit.ros.org
[3]: http://wiki.ros.org/urdf/XML.
[4]: https://ros-planning.github.io/moveit_tutorials/
[5]: https://sir.upc.edu/projects/rostutorials/index.html
[6]: https://industrial-training-master.readthedocs.io/en/melodic/_source/setup/PC-Setup---ROS.html
