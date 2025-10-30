# Composite CBF Point Cloud Selector

This repository contains the ROS point cloud selector for the [Composite CBF Safety Filter](https://github.com/ntnu-arl/composite_cbf).

The node takes as input a depth measurement (point cloud or image) from a sensor, downsamples it via angular binning, and publishes the resulting point cloud.  
It can handle several sensor simultaneously, and output a single, concatenated point cloud.

The sensors characteristics are described in a config file, detailled in [hereafter](#config).

## Installation

### Requirements

* ROS 2 (Tested on Humble)
* Eigen3

### Installation

Simply clone the repo in the `src` folder of your workspace and install with Catkin.

## Config

The sensors are described in a config file, according to the input data type.  
The node supports *depth images*, *range images*, and *point clouds*.

The `yaml` config file for a set of sensors is structured as:

```yaml
frame_body: body_frame  # TF frame for body
sensors:
    -  # depth image
        is_pointcloud: false  # set to false for depth image
        is_polar: false  # set to false for depth image
        topic: /camera_1/depth  # ROS topic for input data
        frame: camera_1_optical  # TF frame for input data
        mm_resolution: 1000  # mm resolution in 1 pixel unit in the input data
        bins_w: 8  # number of output bins (width)
        bins_h: 6  # number of output bins (height)
        percentile: 20  # percentile filter [%] -- selects the N%-th value in each bin
        min_per_bin: 0  # minimum number of points per bin below which the value is disregarded
        min_range: 0.1  # minimum range considered [m]
        max_range: 10  # maximum range considered [m]
        # if camera info topic is available, use:
        cam_info_topic: /camera_1/camera_info  # ROS topic for camera info
        # if camera info topic is not available, comment the line above and use:
        hfov: 56  # halved horizon FoV [deg]
        vfov: 44  # halved vertical FoV [deg]
        image_w: 224  # input image size (width) -- used only for preallocating bin size
        image_h: 172  # input image size (height)-- used only for preallocating bin size
    -  # range image
        is_pointcloud: false  # set to false for depth image
        is_polar: true  # set to true for depth image
        topic: /camera_2/range  # ROS topic for input data
        frame: camera_2_optical  # TF frame for input data
        mm_resolution: 1  # mm resolution in 1 pixel unit in the input data
        bins_w: 8  # number of output bins (width)
        bins_h: 6  # number of output bins (height)
        percentile: 20  # percentile filter [%] -- selects the N%-th value in each bin
        min_per_bin: 100  # minimum number of points per bin below which the value is disregarded
        min_range: 0.6  # minimum range considered [m]
        max_range: 10  # maximum range considered [m]
        azimuth_range: [-180, 180]  # range of azimuth covered by sensor [deg]
        elevation_range: [-45, 45]  # range of elevation covered by sensor [deg]
        nb_pts: 65536  # upper bound for number of lidar points -- used only for preallocating bin size
    -  # point cloud
        is_pointcloud: true  # set to true for point cloud
        topic: /lidar/points  # ROS topic for input data
        frame: lidar_frame  # TF frame for input data
        bins_w: 20  # number of output bins (width)
        bins_h: 10  # number of output bins (height)
        percentile: 20  # percentile filter [%] -- selects the N%-th value in each bin
        min_per_bin: 20  # minimum number of points per bin below which the value is disregarded
        min_range: 0.4  # minimum range considered [m]
        max_range: 10  # maximum range considered [m]
        azimuth_range: [-180, 180]  # range of azimuth covered by sensor [deg]
        elevation_range: [0, 90]  # range of elevation covered by sensor [deg]
        nb_pts: 10000  # upper bound for number of lidar points -- used only for preallocating bin size
```

## Cite

If you use this work in your research, please cite the following publication:

```bibtex
@INPROCEEDINGS{harms2025safe,
  AUTHOR={Marvin Harms and Martin Jacquet and Kostas Alexis},
  TITLE={Safe Quadrotor Navigation using Composite Control Barrier Functions},
  BOOKTITLE={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  YEAR={2025},
  URL={https://arxiv.org/abs/2502.04101},
}
```

or, if you use our embedded implementation, please cite:

```bibtex
@INPROCEEDINGS{misyats2025embedded,
  AUTHOR={Misyats, Nazar and Harms, Marvin and Nissov, Morten and Jacquet, Martin and Alexis, Kostas},
  TITLE={Embedded Safe Reactive Navigation for Multirotors Systems using Control Barrier Functions},
  BOOKTITLE={2025 International Conference on Unmanned Aircraft Systems (ICUAS)},
  pages={697--704},
  YEAR={2025},
  URL={https://arxiv.org/abs/2504.15850},
}
```

## Acknowledgements

This work was supported by the European Commission Horizon Europe grants DIGIFOREST (EC 101070405) and SPEAR (EC 101119774).

## Contact

* [Martin Jacquet](mailto:marvin.jacquet@ntnu.no)
* [Marvin Harms](mailto:marvin.c.harms@ntnu.no)
* [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
