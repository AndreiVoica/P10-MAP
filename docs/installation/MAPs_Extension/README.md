## How to install MAPs Extension


1. Copy the *maps* folder into the Isaac Sim extensions folder. The default path in Ubuntu (Isaac Sim 2022.2.1) is: 
```
/home/$USER$/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/maps
```

2. In the *extensions.toml* file, add the following lines:
```
PATH:
/home/$USER$/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/config/extensions.toml

TO ADD:
[[python.module]]
name = "omni.isaac.examples.maps"
```

3. Once the extension is installed, it is necessary to launch the ROS master in a terminal using `roscore` before opening Isaac Sim GUI.

4. Then you can run the MAPs extension from the Isaac Examples tab in the Isaac Sim GUI:

![MAPs Extension](/docs/imgs/MAPs_extension_menu.jpg)

### Known Issues

The first time you load the Isaac Sim GUI, the MAPs extension doesn't appear, to fix this just go to the *maps.py* file and save it again. This will load the files and allow you to launch it from the GUI.

```
/home/$USER$/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/maps/maps.py
```
