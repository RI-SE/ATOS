# PointCloudPublisher

## About the module
`PointcloudPublisher` is used to publish site scans. This module supports pointclouds that have the file type `.pcd`. If you have a very large pointcloud, it is recommended to downsample it before inputting it into the module.

## ROS parameters
The following ROS parameters can be set for `PointcloudPublisher`:

```yaml
atos:
  pointcloud_publisher:
    ros__parameters:
      pointcloud_files: ["file1.pcd", "file2.pcd"]     # List of one or more pointcloud files to publish.
```


