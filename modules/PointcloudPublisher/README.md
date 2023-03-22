# About the module
Short description what the module does and what to use it for.

# ROS parameters
The follwing ROS parameters can be set for `PointcloudPublisher`:
- `pointcloud_files` - A list of all file names of pointclouds to load, the pointcloud must be a `.pcd`-file and be located in the `pointclouds`-directory.

## Examples
### Example 1
Load a single pointcloud:
- `pointcloud_files: ["Pointcloud.pcd"]`

### Example 2
Load multiple pointcloud files:
- `pointcloud_files: ["Pointcloud1.pcd", "Pointcloud2.pcd"]`
