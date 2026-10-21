# pointcloud_to_grid

A ROS 2 (Humble) node that converts `sensor_msgs/PointCloud2` data into 2D grids,
published both as `nav_msgs/OccupancyGrid` and as `grid_map_msgs/GridMap`.

This is a derivative of [jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid).
See the upstream repository for the original project, its sample rosbag and the conventions figure.

![](doc/grid_map01.gif)

## What is different from upstream

- **Any field as the grid value**: `value_field` selects `z`, a colour channel or any float field.
- **Cell retention**: `retention_time` keeps cells for a number of seconds instead of
  rebuilding the grid from every cloud.
- **Dual output**: OccupancyGrid and GridMap are published side by side.
- **Refactored core**: the point and grid logic lives in header files with unit tests.
- **Paralell execution**: this node handles one source. For several sources, run one
  instance per source and fuse the results e.g. with the `grid_fusion` package.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select pointcloud_to_grid --symlink-install
source install/setup.bash
```

Dependencies: `rclcpp`, `sensor_msgs`, `nav_msgs`, `grid_map_msgs`, `visualization_msgs`,
`tf2*`, `pcl_ros`, `pcl_conversions`.

## Run

```bash
ros2 launch pointcloud_to_grid demo.launch.py topic:=my_cloud_topic
ros2 launch pointcloud_to_grid rviz.launch.py
```

Or start the node directly:

```bash
ros2 run pointcloud_to_grid pointcloud_to_grid_node --ros-args \
  -p cloud_in_topic:=/lidar/points -p value_field:=intensity -p intensity_factor:=100.0
```

## Topics

| Direction | Default topic | Type |
|---|---|---|
| in | `nonground` (`cloud_in_topic`) | `sensor_msgs/PointCloud2`, best-effort QoS |
| out | `intensity_grid` | `nav_msgs/OccupancyGrid` |
| out | `height_grid` | `nav_msgs/OccupancyGrid` |
| out | `intensity_gridmap` | `grid_map_msgs/GridMap`, layer `intensity` |
| out | `height_gridmap` | `grid_map_msgs/GridMap`, layer `height` |

The input uses `BEST_EFFORT` reliability, which matches most LiDAR drivers and avoids QoS
warnings. The output frame is the `frame_id` of the incoming cloud. The node does not transform
between frames.

## Parameters

All parameters can be changed at runtime. Changing a topic name recreates the publisher or
subscription, and changing the grid geometry clears the retained cells.

| Parameter | Default | Description |
|---|---|---|
| `cloud_in_topic` | `nonground` | Input point cloud topic. |
| `value_field` | `intensity` | Where the cell value comes from, see below. |
| `cell_size` | `0.5` | Cell edge length in metres. |
| `position_x`, `position_y` | `0.0` | Grid centre in metres. |
| `length_x`, `length_y` | `20.0`, `30.0` | Grid size in metres. |
| `intensity_factor` | `1.0` | Multiplier applied to the value for the intensity grid. |
| `height_factor` | `1.0` | Multiplier applied to the value for the height grid. |
| `retention_time` | `0.0` | Seconds a cell is kept. `0` means every cloud replaces the grid. Must be `>= 0`. |
| `mapi_topic_name`, `maph_topic_name` | `intensity_grid`, `height_grid` | OccupancyGrid output topics. |
| `mapi_gridmap_topic_name`, `maph_gridmap_topic_name` | `intensity_gridmap`, `height_gridmap` | GridMap output topics. |
| `verbose1`, `verbose2` | `true`, `false` | Log a line for every published grid, and extra debug output. |

### `value_field`

| Value | Source in the cloud |
|---|---|
| `z` or `height` | The `z` field. |
| `r`, `g`, `b` | One colour channel, unpacked from the packed `rgb` field (0 to 255). |
| anything else | A float field of that name, for example `intensity` or `reflectivity`. |

If the cloud has no `x`/`y` field, or no field matching `value_field`, the frame is skipped and
an error lists the fields the cloud does have.

## How cells are filled

- **Both grids use the same value.** Per point, the intensity cell is `value * intensity_factor`
  and the height cell is `value * height_factor`. Both are cast to a signed byte, so keep
  `value * factor` within -127 to 127. A value of 0.8 with `intensity_factor: 100.0` becomes 80.
  With the default factor of 1.0 a 0 to 1 intensity truncates to 0.
- **The last point in a cell wins.**
- **Unknown cells** hold `-128`. In GridMap messages every cell holds `cell / 127`, so unknown is
  about -1.008.
- **Skipped points:** points outside the grid area (edges exclusive) and points with
  `|x| <= 0.01 m`.
- **Layout:** cell `(0, 0)` is the top-left corner (maximum x and y). Indices grow towards -x
  and -y, and data is stored row by row. The OccupancyGrid origin sits at that corner and is
  rotated 180° about z.

### Retention

With `retention_time > 0`, each cell remembers when it was last written, using the cloud's
`header.stamp` (or the node clock if the stamp is zero). Cells older than `retention_time` are
dropped before each cloud is applied. Cells stamped in the future, for example when a looping
bag restarts, are dropped as well.

## Several sources

Run one instance per source with a unique node name and unique topic names. The grid parameters
must be identical across instances if you want to fuse the results. The `playground_bringup`
package starts this setup from one launch file, and `grid_fusion` combines the outputs.

## Launch files

| File | Purpose |
|---|---|
| `demo.launch.py` | Single node, argument `topic`. |
| `dual_output_example.launch.py` | Single node with geometry and topic arguments. |
| `grid_trajectory.launch.py` | Starts the separate `grid_trajectory` executable (not covered here). |
| `rviz.launch.py` | RViz2 configuration. |

## Tests

```bash
colcon test --packages-select pointcloud_to_grid
colcon test-result --all --verbose
```

Unit tests cover the grid mapping, value fields, retained cells, point accumulation and message
filling.

## Code layout

| File | Content |
|---|---|
| `pointcloud_to_grid_core.hpp` | `GridMap` geometry and message initialisation. |
| `value_reader.hpp` | `ValueSource` and `ValueReader`: which field to read. |
| `grid_processing.hpp` | Cloud validation, `RetainedGrid`, point accumulation, message filling. |
| `src/pointcloud_to_grid_node.cpp` | The node: parameters, subscription, publishing. |

## License and credit

Derived from [jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid).
At the time of this fork the upstream project used BSD-3-Clause. So does this Project for now.