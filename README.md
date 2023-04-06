# Indy ROS2 Package

ROS2 integration for Neuromeka research manipulators

1. [indy\_description](#indy_description)
2. [indy\_gazebo](#indy_gazebo)

## indy_description

We performed approximate convex decomposition from the visual mesh using [V-HACD](https://github.com/kmammou/v-hacd) to generate effective collision meshes.
And we used [meshlab](https://www.meshlab.net/) to convert file formats.

```sh
# Install V-HACD
cd ${YOUR_GIT_DIR}
git clone https://github.com/kmammou/v-hacd.git
cd ${YOUR_GIT_DIR}/v-hacd/app
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Add alias to .bashrc or .zshrc
alias TestVHACD='${YOUR_GIT_DIR}/v-hacd/app/build/TestVHACD'

# Install meshlab
sudo apt install meshlab
```

You can create a collision mesh `*.obj` from a visual mesh `*.stl` using the following commands:

```sh
# Convert STL to Wavefront OBJ
meshlabserver -i <visual_mesh.stl> -o <wavefront.obj>

# Approximate convex decomposition
TestVHACD <wavefront.obj> -h <n> -l 5
```

You can also use `generate_collision_mesh.sh` in the `meshes` directory for convenience.
Before using it, you have to edit the variable `testvhacd_executable` in the script to your `TestVHACD` executable path.

```sh
./generate_collision_mesh.sh -i <visual_mesh.stl> <parameters>
# This will generate your_mesh.obj
```

## indy_gazebo

This package is for Gazebo, a.k.a. Ignition, version above Fortress, not for Gazebo Classic.
We recommend installing `ros_gz` from the `ros2` branch in the [source code](https://github.com/gazebosim/ros_gz).
Since `apt install ros-humble-ros-gz` may ignore the Gazebo version you are currently using and install its own separate Gazebo and libraries, potentially causing complications.
Some libraries that come with `apt install` still have names that have not been entirely changed from `ign` to `gz`.

To install `ros_gz` from source code, follow the instructions below:

```sh
# Download needed software
cd ${YOUR_WS}/src
git clone https://github.com/gazebosim/ros_gz.git -b ros2

# Build
cd ${YOUR_WS}
rosdep install -r --from-paths src -i -y --rosdistro ${ROS_DISTRO}
colcon build
```















<!-- rosrun xacro xacro `rospack find foo`/urdf/foo.xacro > foo.urdf
ign sdf -p foo.urdf > foo.sdf
ign sdf -k foo.sdf
Valid.
ign gazebo -r foo.sdf


https://answers.ros.org/question/387494/how-to-convert-a-gazebo-ros-system-to-ignition/ -->