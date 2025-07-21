# acg_resources_fanuc_m20ia_35m_description

The URDF provided in this package has been created from the information provided in the [official Fanuc M-20iA/35M datasheet](https://www.fanucamerica.com/cmsmedia/datasheets/M-20iA_35M%20Product%20Information_299.pdf) and the [official Fanuc M-20iA operator's manual](https://www.migatronic.com/media/1384/manual_am-120ic_operator_manual_b-82874en_07.pdf).
Position, velocity and effort limits were directly taken from said datasheet, with the exception of the effort limits for joints 1, 2 and 3, which were chosen arbitrarily high as this information is not provided by the manifacturer.
The joints' range of motion was taken form the operator's manual.

## Usage

In order to visualize the Fanuc M-20iA/35M robot described in the URDF, a launch file is provided with this package.
After building, launch it with the following command:

```bash
ros2 launch acg_resources_fanuc_m20ia_35m_description display.launch.py
```

RViz will be launched and the Fanuc robot visualized together with its reference frames.
The Joint State Publisher's GUI can be used to move single joints.

To specify a robot's prefix name different from the default `fanuc_m20ia_35m/`, use the `tf_prefix` launch argument. In this case, [urdf.rviz](config/urdf.rviz) must be manually updated.

## Robot meshes

The [meshes](./meshes/) directory contains [collision](./meshes/collision/) and [visual](./meshes/visual/) objects.
Visual objects include `.dae` (COLLADA) meshes, used in the URDF `visual` tags, while collision objects include `.stl` meshes used in the URDF `collision` tags.
Both visual and collision meshes have been originally exported from a CoppeliaSim scene built from CAD files.
To solve a visualization issue in RViz, the COLLADA visual meshes were imported in Blender and re-exported in the same format.
This process did not apply any transform to the original meshes.
As Blender does not support exporting COLLADA meshes with the `<up_axis>` tag set to `Y_UP` like in the original meshes, this tag was manually edited at the end of this process.
