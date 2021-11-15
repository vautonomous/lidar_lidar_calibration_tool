# How to use
Rviz automatically will be seen on your screen for all modes. Don't forget to set fixed frame target_frame or map frame (only Mode 3).
## Mode 0:
It stores the synced point cloud pairs as .pcd and .ply files into a folder. Set those parameters in order to launch the Mode 0.


**Parameters:**

```bash
topic_name_target  = "/velodyne_points_raw"
topic_name_source  = "/vlp16_01/velodyne_points"
path_pcd_folder    = "/home/.../folder"
```
Then,

```bash
roslaunch lidar_lidar_calibration_tool calibrate.launch
rosbag play rosbag.bag
```
## Mode 1:
Single run registration with target and source .pcd files.

```bash
trans_initial_alignment      =  [-0.65, -0.76, 0, -2.22,
                                0.76, -0.65, -0.05, 0.06,
                                0.04, -0.03, 1, -1.7,
                                0, 0, 0, 1]
path_pcd_target              = "/home/goktug/projects/lidar_lidar_calibration_tool/src/maps/3_map_target.pcd"
path_pcd_source              = "/home/goktug/projects/lidar_lidar_calibration_tool/src/maps/3_map_source.pcd"
frame_id_target_single_run   = "velodyne"
<!-- Applying passthrough filter on the both target and source clouds, before GICP, after initial aligment-->
pass_through_min_x           = "-150"
pass_through_max_x           = "150"
pass_through_min_y           = "-150"
pass_through_max_y           = "150"
pass_through_min_z           = "-150"
pass_through_max_z           = "150"
<!-- Before GICP -->
leaf_single_run_target       = "0.2"
leaf_single_run_source       = "0.1"                
```




## Mode 2:
It applies the GICP registration on the point cloud pairs which are stored into a file by launching the Mode 0. It returns the mat_source_to_target which has the best fitness score, by iterating whole point cloud pairs.

**Note that:** You must start Mode 0, before starting Mode 2.

```bash
topic_name_target  = "/velodyne_points_05"
topic_name_source  = "/velodyne_points_01"
path_pcd_folder    = "/home/.../folder"
trans_initial_alignment =  [-0.65, -0.76, 0, -2.22,
                          0.76, -0.65, -0.05, 0.06,
                            0.04, -0.03, 1, -1.7,
                            0, 0, 0, 1]
leaf_size_target = "0.3"
leaf_size_source = "0.1"
frame_id_target  ="velodyne"
```
```bash
roslaunch lidar_lidar_calibration_tool calibrate.roslaunch
```
## Mode 3:
It generates a map by using leo_slam NDT mapping mode. In order to start this mode, you don't have to start mode 0, just give a rosbag path. Also, it stores the map into a folder as a .ply and .pcd file, after each iteration.
```bash
path_bag_file       = "/home/goktug/Desktop/bagfiles/egimli/korkunc_sonsuz_egim.bag"
topic_cloud_to_map  = "/velodyne_points_raw"
leaf_size_map       = "0.4" <!-- Downsampling on map after each iteration -->
path_map_save       = "/home/goktug/projects/lidar_lidar_calibration_tool/src/maps"

bool_radius_filter  = "true"/> <!-- On map after each iteration -->
radius              = "0.8"
minNeighbors        = "2"

bool_sor_filter     = "true"/> <!-- On map after each iteration -->
num_neigbor_points  = "10"
std_dev             = "1.0"
```
Firstly, launch the leo_slam:
```bash
roslaunch leo_slam leo_slam.launch
roslaunch lidar_lidar_calibration_tool calibrate.roslaunch
```

