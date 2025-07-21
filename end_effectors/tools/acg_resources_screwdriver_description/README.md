# acg_resources_screwdriver_description

This package contains description files for a scredriver tool.

## Visualization demo

This package provides a launch file to visualize the scredriver in RViz.

To visualize the scredriver in RViz, run the following command:

```bash
ros2 launch acg_resources_screwdriver_description display.launch.py
```

## Xacro Macro Parameters

The main macro in this package, [`build_tool`](urdf/screwdriver.urdf.xacro), accepts several parameters.
For a general explanation of these parameters, please refer to the [tools container README](../README.md#how-to-extend-the-tools-library).

The `config` parameter is a dictionary that can be used to pass optional or necessary configuration keys to the macro.
For the screwdriver, the following key is currently supported:

* `rpy`: The RPY orientation (in radians) of the screwdriver with respect to the mounting frame.
    * **Type:** string (e.g., `"0 0 0"`)
    * **Default:** `"0 0 0"`
    * **Usage:** If not specified, the default orientation is used.

## Origin of the meshes

The visual and collision meshes used in this package have been produced via SketchUp and later modified in Blender.
