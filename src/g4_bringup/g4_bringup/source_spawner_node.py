#!/usr/bin/env python3
"""
ROS 2 node to spawn visual markers for Cs‑137 sources in Gazebo.

This node reads a list of source positions from the parameter
`source.positions` (a flat list of [x,y,z] triples in metres) and,
after a short delay, spawns a sphere model at each position using the
`gz service` command.  The sphere model is defined in an SDF file
whose path can be provided via the `sdf_file` parameter; if not
specified, a default path within the `g4_bringup` package is used.

Example parameter YAML snippet:

```yaml
source.positions: [1.0, 0.0, 0.5, 2.0, -1.0, 0.5]
sdf_file: "/home/user/g4_ws/src/g4_bringup/models/cs137_sphere.sdf"
```

This node is intended to be launched alongside Gazebo after the world
has been created.  It delays spawning by 2 seconds to ensure the
`/world/<name>/create` service is available.
"""

import rclpy
from rclpy.node import Node

import subprocess
import os
from ament_index_python.packages import get_package_share_directory


class SourceSpawner(Node):
    def __init__(self) -> None:
        super().__init__('source_spawner')
        # Declare parameters
        self.declare_parameter('source.positions', [])
        self.declare_parameter('sdf_file', '')
        self.declare_parameter('world_name', 'minimal')
        # Schedule spawning after a short delay (2 seconds).  The timer
        # automatically cancels itself once spawning is complete.
        self.timer = self.create_timer(2.0, self.spawn_sources)

    def spawn_sources(self) -> None:
        # Retrieve the source positions parameter.  rclpy stores arrays of
        # doubles as Python lists, so we can use them directly.  However,
        # depending on the type of parameter value, we may need to
        # extract the value field.
        raw_positions = self.get_parameter('source.positions').get_parameter_value()
        # The parameter may be a double array (ParameterValue.double_array_value)
        if hasattr(raw_positions, 'double_array_value') and raw_positions.double_array_value:
            positions_list = list(raw_positions.double_array_value)
        else:
            positions_list = raw_positions if isinstance(raw_positions, list) else []

        if not positions_list:
            self.get_logger().info('No source.positions parameter provided; nothing to spawn.')
            self.timer.cancel()
            return

        if len(positions_list) % 3 != 0:
            self.get_logger().error(
                f'source.positions length {len(positions_list)} is not divisible by 3; expected triples.')
            self.timer.cancel()
            return

        # Determine the path to the SDF file used for the spheres.  If the
        # user provided a custom sdf_file parameter, use it.  Otherwise,
        # attempt to locate cs137_sphere.sdf within the g4_bringup package.
        sdf_param = self.get_parameter('sdf_file').get_parameter_value()
        if hasattr(sdf_param, 'string_value') and sdf_param.string_value:
            sdf_path = sdf_param.string_value
        else:
            # Use default path: <pkg_share>/models/cs137_sphere.sdf
            try:
                pkg_share = get_package_share_directory('g4_bringup')
                sdf_path = os.path.join(pkg_share, 'models', 'cs137_sphere.sdf')
            except Exception:
                # Fall back to current working directory
                sdf_path = os.path.join(os.getcwd(), 'cs137_sphere.sdf')
        if not os.path.isfile(sdf_path):
            self.get_logger().error(f'Cannot find SDF file at {sdf_path}.')
            self.timer.cancel()
            return

        # Get world name for the service path (e.g. /world/minimal/create)
        world_param = self.get_parameter('world_name').get_parameter_value()
        world_name = world_param.string_value if hasattr(world_param, 'string_value') and world_param.string_value else 'minimal'

        # Iterate over each source and spawn a sphere using gz service
        for i in range(0, len(positions_list), 3):
            x, y, z = positions_list[i], positions_list[i + 1], positions_list[i + 2]
            name = f'cs137_source_{i // 3}'
            # Construct the service request string.  Quotation marks and
            # curly braces must be carefully balanced to satisfy the
            # proto3 formatted string expected by Gazebo.  We include a
            # pose element with position only; orientation defaults to
            # identity.
            req = (
                f'sdf_filename: "{sdf_path}" '
                f'name: "{name}" '
                f'pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }} '
                f'allow_renaming: false'
            )
            cmd = [
                'gz', 'service',
                '-s', f'/world/{world_name}/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--req', req,
            ]
            self.get_logger().info(f'Spawning source sphere: {name} at ({x}, {y}, {z})')
            try:
                subprocess.run(cmd, check=True)
            except Exception as e:
                self.get_logger().error(f'Failed to spawn {name}: {e}')
        # All sources spawned; cancel the timer
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SourceSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()