# AMBF Load ADF Plugin

A plugin for the Asynchronous Multibody Framework (AMBF), see https://github.com/WPI-AIM/ambf
The plugin allows you to load an ADF file into the AMBF simulator environment at arbitrary times during runtime using a ros service call

Developed by Henry Phalen

This exists as a "drag-and-drop" feature in the simulator already, but it is nice to be able to add things programmatically as well (e.g. you could call this service from e.g. a Python script to load an ADF file at a certain time in a simulation)

## Features:

1. Load an ADF file into the simulator at runtime using a ros service call

``` rosservice call /ambf/load_adf_plugin/add_adf_file "data: '<filename>'" ```


## Building
Find most updated instructions for building AMBF on that git repository (https://github.com/WPI-AIM/ambf)

This plugin uses ROS so you build it in a catkin workspace. 

### Build within a catkin workspace (ROS1)
A simple ```package.xml``` has been included in this repository to enable catkin to find and build it
```<plugin_path>``` should be located within ```catkin_ws/src/```
```
cd <catkin_ws>
catkin build ambf_load_adf_plugin
```

## Using plugin

### Starting plugin
Find most updated instructions for starting AMBF with plugins on that git repository (https://github.com/WPI-AIM/ambf)

You can test this plugin on the example by:
```<ambf_exe_dir> ---> e.g. ~/ambf/bin/lin-x86_64```
```<plugin_so_path> ---> e.g. <catkin_ws>/devel/lib/libambf_load_adf_plugin.so```
```<adf_filepath> ---> e.g. <ambf_path>/ambf_models/descriptions/multi-bodies/robots/blender-toy-car.yaml```

Start a ROS core. Then start the simulator with the plugin:

```bash
cd <ambf_exe_dir>
./ambf_simulator --plugins <plugin_so_path>
```

Then you can load the ADF file into the simulator using the service call:
```bash
rosservice call /ambf/load_adf_plugin/add_adf_file "data: '<adf_filepath>'"
```

## TODOs / Future Work
Something I hope to get to someday (or you could do it too! :) )

1. Why does the so appear in ```<catkin_ws>/devel/lib```, not ```<catkin_ws>/build/```?
1. ROS2 version