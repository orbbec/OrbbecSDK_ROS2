# Component node

In ROS2, component nodes enable efficient resource management and modularity by allowing multiple nodes to be loaded into a single process, called a component container. Here’s an overview of setting up a component node and adding it to a container, both for a single node and through a `launch.py` file.

### Creating a component node

To define a node as a component, the following steps are required:

1. **Create a node class**: The node class should inherit from `rclcpp::Node` (C++) or `Node` (Python) and implement the core functionalities within it.
2. **Add plugin export**: In `CMakeLists.txt`, export the node as a plugin by adding it to a component library using `rclcpp_components`. For example:

   ```cmake
   # CMakeLists.txt
   find_package(rclcpp_components REQUIRED)

   add_library(my_component SHARED src/my_component.cpp)
   target_link_libraries(my_component PUBLIC rclcpp::rclcpp)
   ament_target_dependencies(my_component rclcpp rclcpp_components)

   rclcpp_components_register_nodes(my_component "mypackage::MyComponent")
   ```
3. **Declare the plugin in package.xml**: Add the node as a plugin in the package manifest:

```xml
  <export>
    <rclcpp_components>
      <node plugin="mypackage::MyComponent" />
    </rclcpp_components>
  </export>
```

### Loading a single node into a component container

To load a node directly into a component container, use the ComponentManager node. Run the command:

```bash
  ros2 run rclcpp_components component_container
```

Then load the component into this container with the load_component service:

```bash
  ros2 component load /ComponentManager mypackage my_component
```

Replace /ComponentManager with the actual name of your container, if different.
3. Adding Component Nodes via a Launch File
In launch.py, you can define a component container and load nodes into it. Here’s an example launch.py file:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # use 'component_container' for single-threaded
        composable_node_descriptions=[
            ComposableNode(
                package='mypackage',
                plugin='mypackage::MyComponent',
                name='my_component'
            ),
            ComposableNode(
                package='another_package',
                plugin='another_package::AnotherComponent',
                name='another_component'
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

**explanation**

- ComposableNodeContainer: This creates a component container to hold nodes. Use component_container_mt for multi-threading or component_container for - single-threaded operation.
- ComposableNode: Specifies each component to load, with arguments for the package name, plugin type, and node name.
- output: Set to 'screen' to display output in the terminal.
  Running the Launch File
  To run the launch file, use the command:

```bash
ros2 launch mypackage my_launch_file.launch.py
```

This starts the component container and loads the specified nodes into it, enabling efficient component management in ROS2.

### Loading a launch.py into a component container

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    bringup_dir = os.path.join(get_package_share_directory('mypackage'))

    # Define the shared container name
    shared_container_name = "shared_nvblox_container"

    # Create the shared component container
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',  # or 'component_container' for single-threaded
        output='screen'
    )

    # Include another launch file to attach nodes to the shared container
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'orbbec.launch.py')]),
        launch_arguments={
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('from_bag'))
    )

    # Declare any required launch arguments
    from_bag_arg = DeclareLaunchArgument(
        'from_bag',
        default_value='false',
        description='Condition to use data from a bag file'
    )

    # Return LaunchDescription with shared container and nodes attached
    return LaunchDescription([from_bag_arg, shared_container, orbbec_launch])
```

**Explanation**

- **shared_container_name**: The name of the shared container, which other nodes can reference for attaching.
- **shared_container**: Defines the shared container as a `Node`, using `component_container_mt` for multi-threading. This container will host multiple component nodes.
- **IncludeLaunchDescription**: Loads and attaches nodes from another launch file (in this example, orbbec.launch.py) to the shared container.
  - launch_arguments: The arguments passed to the included launch file.
    - **attach_to_shared_component_container**: Set to `'True'`, specifying that nodes in `orbbec.launch.py` should be added to the existing shared container.
    - **component_container_name**: References the `shared_container_name`, linking nodes from the included launch file to the shared container.
  - **condition**: Only includes the `orbbec.launch.py` nodes in the shared container if the `from_bag` parameter is `false`.

**Running the launch File**

Execute the following command to start the launch file:

```
ros2 launch mypackage my_main_launch_file.launch.py
```

This command will start the shared component container and attach the nodes specified in `orbbec.launch.py` to it if the `from_bag` condition is not met.
