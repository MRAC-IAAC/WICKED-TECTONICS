# WICKED TECTONICS
*ENABLING ROBOTIC OPERATIONS IN ILL-DEFINED DOMAINS*

MRAC02 RS.2 - Advanced Technology
![](https://lh4.googleusercontent.com/SYoaxkknYLGyJcnypk4rCv4iGYDB-tmlDNaeVDsWxJ-tV9ybrGcn7AfaLtT0KlLpKQV0-Zuxx4B5i-4AbfTM4w1e2IRQqDYePFscEsYvqfaTJ-lHHEvTmIx_h0tFExKjRxTnbF51)
In the context of the current paradigm of Industry 4.0, production processes are becoming more and more informed by multiple streams of data of different levels and modalities.  
For this seminar, we aim to explore the integration of sensors in processes involving a UR10 6-Axis industrial robots, with the goal of controlling the handling and manipulation of materials, objects and environments with limited prior knowledge about their physical properties or geometry: they are, in this sense, “wicked”.

In such conditions, the information necessary to operate in a controlled way needs to be acquired either before or during the process (i.e. extracted from raw data that is itself captured by sensors), and then integrated in the program that is controlling the machine, to achieve its goals.

Translating this into a working technical implementation therefore requires linking hardware and software components at several levels, such as sensing and processing, computational design,  control & simulation, and physical manipulation.

In this seminar, we aim to set up a working robotic system that is able to cope with “wicked” domains, by implementing and integrating the necessary technical building blocks on all levels of such a system. The generic setup will then be customized and applied in the context of the individual thesis projects.

**Learning Objectives**

By the end of this seminar, students will have acquired the necessary skills and concepts to be able to:

-   Conceptualize and design a robotic fabrication or assembly system with sensor integration in architecture- and construction-related scenarios
    
-   Use the tools necessary to design and communicate the anatomy of such a system
    

Implement it, by integrating the required sensing, processing, design, simulation, control and manipulation tools

Faculty – Raimund Krenmüller, Angel Muñoz, Soroush Garivani

## WORKFLOW

![Workflow](/images/diagram.jpg)

The robotic application developed for this workshop is based on [Compas Fab](https://gramaziokohler.github.io/compas_fab/latest/). A **Robotic fabrication package for the COMPAS Framework** that facilitates the planning and execution of robotic fabrication processes. It provides interfaces to existing software libraries and tools available in the field of robotics (e.g. OMPL, ROS) and makes them accessible from within the parametric design environment. The package builds upon [COMPAS](https://compas.dev/), an open-source Python-based framework for collaboration and research in architecture, engineering and digital fabrication.

In our application we use Rhino and Grasshopper as CAD software interfacing through a websocket with the robotic platform ROS, Universal Robots ROS driver to control the robot, Moveit as motion planner and a Realsense D435 RGBD camera as sensor to detect the pieces contours using the openCV library.


The system is divided in 3 main parts, 2 computers and a Universal Robots UR10e. 


## WINDOWS COMPUTER

### Requirements

* Minimum OS: Windows 10 Pro
* [Anaconda 3](https://www.anaconda.com/distribution/)
* [Rhino 6/7 & Grasshopper](https://www.rhino3d.com/download)
* [Visual Studio Code](https://code.visualstudio.com/): Any python editor works, but we recommend VS Code + extensions [as mentioned in the compas fab documentation](https://gramaziokohler.github.io/compas_fab/latest/getting_started.html#working-in-visual-studio-code-1)

### Compas Fab installation

We use `conda` to make sure we have clean, isolated environment for dependencies. Open Anaconda Prompt.

If it's your first time using conda make sure you run this at least once:

    (base) conda config --add channels conda-forge

Create a new conda environment and activate it:

    (base) conda create -n rs2 python=3.8 compas_fab
    (base) conda activate rs2

### Verify installation

        (rs2) python -m compas
    Yay! COMPAS is installed correctly!
    
    COMPAS: 1.7.1
    Python: 3.8.10 | packaged by conda-forge | (default, May 11 2021, 06:25:23) [MSC v.1916 64 bit (AMD64)]
    Extensions: ['compas-fab']

### Install on Rhino

    (rs2) python -m compas_rhino.install

    
## UBUNTU 18.04 COMPUTER

### Requirements

* OS: [Ubuntu 18.04](https://releases.ubuntu.com/18.04.5/) - [Installation guide](https://phoenixnap.com/kb/how-to-install-ubuntu-18-04)

### Setting up Ubuntu with a real-time kernel
In order to run the `universal_robot_driver`, we highly recommend to setup a ubuntu system with real-time capabilities. Especially with a robot from the e-Series the higher control frequency might lead to non-smooth trajectory execution if not run using a real-time-enabled system.

Follow this [guide](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md) in order to compile the ubuntu real-time kernel

### ROS melodic full desktop installation

#### Configure your Ubuntu repositories
 Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can [follow the Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.

#### Setup your sources.list
Setup your computer to accept software from packages.ros.org.

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#### Set up your keys

    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#### ROS melodic installation

    sudo apt update
    sudo apt install ros-melodic-desktop-full

#### Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
#### Create a ROS Workspace

Let's create and build a [catkin workspace](http://wiki.ros.org/catkin/workspaces):

    mkdir -p ~/ur_ws/src
    cd ~/ur_ws/
    catkin_make
    source devel/setup.bash

#### Rosbridge installation

Rosbridge provides a JSON API to ROS functionality for non-ROS programs. There are a variety of front ends that interface with rosbridge, including a WebSocket server for web browsers to interact with. Rosbridge_suite is a meta-package containing rosbridge, various front end packages for rosbridge like a WebSocket package, and helper packages.

    sudo apt-get install ros-melodic-rosbridge-server

#### MoveIt installation
Open-source robotic manipulation framework that allows to develop complex manipulation applications using ROS.

`sudo apt install ros-melodic-moveit`

#### Universal Robots ROS Driver installation
[Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) that allow us to control a UR robot from ROS.

    # source global ros
    $ source /opt/ros/melodic/setup.bash
    
    # create a catkin workspace
    $ mkdir -p ur_ws/src && cd ur_ws
    
    # clone the driver
    $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
    
    # clone fork of the description. This is currently necessary, until the changes are merged upstream.
    $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
    
    # install dependencies
    $ sudo apt update -qq
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src -y
    
    # build the workspace
    $ catkin_make
    
    # activate the workspace (ie: source it)
    $ source devel/setup.bash



## UR robot setup for ur_robot_driver

### Prepare the robot

For using the _ur_robot_driver_ with a real robot you need to install the **externalcontrol-1.0.4.urcap**

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is necessary.

For installing the necessary URCap and creating a program, please see the individual tutorial on how to [setup an e-Series robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md).

### Prepare the ROS PC (Ubuntu 18.04 computer)

For using the driver make sure it is installed.

#### Extract calibration information

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script:

```
$ roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

```

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As `target_filename` provide an absolute path where the result will be saved to.

#### Quick start

Once the driver is built and the **externalcontrol** URCap is installed on the robot, you are good to go ahead starting the driver. (**Note**: We do recommend, though, to [extract your robot's calibration](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information) first.)

To actually start the robot driver use one of the existing launch files

```
$ roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.56.101

```

If you calibrated your robot before, pass that calibration to the launch file:

```
$ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 \
  kinematics_config:=$(rospack find ur_calibration)/etc/ur10_example_calibration.yaml

```

If the parameters in that file don't match the ones reported from the robot, the driver will output an error during startup, but will remain usable.

For more information on the launch file's parameters see its own documentation.

Once the robot driver is started, load the [previously generated program](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#prepare-the-robot) on the robot panel that will start the _External Control_ program node and execute it. From that moment on the robot is fully functional. You can make use of the _Pause_ function or even _Stop_ (⏹️) the program. Simply press the _Play_ button (▶️) again and the ROS driver will reconnect.

Inside the ROS terminal running the driver you should see the output `Robot ready to receive control commands.`

To control the robot using ROS, use the action server on

/scaled_pos_joint_traj_controller/follow_joint_trajectory

Use this with any client interface such as [MoveIt!](https://moveit.ros.org/) or simply the `rqt_joint_trajectory_controller` gui:

```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

```

You may need to install rqt_joint_trajectory_controller by running:

```
sudo apt install ros-melodic-rqt-joint-trajectory-controller

```

## USAGE



1. **ROS side**
In the Ubuntu 18.04 computer execute one the scripts you will find in the bash_scripts folder.

    `#for just simulation`
    `./ros_compas_fab.sh`
or
    `#to control a real robot`
   ` ./ros_compas_fab_live.sh`

	Both files launch different ROS nodes:

	 - **roscore**, a collection of basic ros nodes like ROS Master, ROS Parameter Server and rosout logging node.
	 - **rosbridge**, to communicate with Grasshopper.
	 - **fileserver**, to exchange files.
	 - **ur_robot_driver**, to control the real robot.(only in the live bash script)
	 - **MoveIt** for the path planning
	 - **cv_basics** to get the pieces contour points and color.

2. **CAD side**
Open the GH file from the cad folder. The Grasshopper definition contains different groups of components, each of them related with a different functionality.

	**ROS client**
	It connects to ROS, just insert the IP of the ROS computer and set to True the connect button. Then load the robot model loading the URDF model from ROS, it creates robot model from URDF and loads the robot geometry.
	
	**Tool**. 
	It sends to MoveIt the tool geometry.
	
	**Planning scene**. 
	It sends to Moveit the scene geometries to avoid collisions.
	
	**Motion Planning**
	Examples of joint and linear motion plannings.

	**Execution**
	This part executes the calculated joint trajectories, from the motion planning examples, in the real robot.
	Subscribe to the /joint_states to get the actual robot joints values and visualize it.

	**Configuration**
	Use this part to home the robot or to move the robot to any desired position using the provided sliders.
	
	**Visualization**
	Display the robot model in the Rhino workspace

3. **UR10e side**

	On the pendant load the URCaps program for external control and execute it by pressing the play button. Inside the ROS terminal running the driver you should see the output 
	`Robot ready to receive control commands.`


## Projects
Applications envisioned and developed by the students during the seminar.

[LINK](http://www.iaacblog.com/programs/courses/mrac02/2020-2021-mrac02/advanced-technology-mrac02-2020-2021-1rt/)


## References

 - https://www.ros.org
 - https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
  - https://opencv.org/ https://www.intelrealsense.com/depth-camera-d435/
   - https://compas.dev/index.html
   - https://gramaziokohler.github.io/compas_fab/latest/
   - https://github.com/compas-dev/compas_fab

## Acknowledges 

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45">

<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

More details: <a href="https://iaac.net/rosin-new-robotic-setup/"> in this link. </a>



















