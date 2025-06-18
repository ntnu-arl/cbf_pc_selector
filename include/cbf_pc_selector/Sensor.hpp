#ifndef CBFPCSEL_SENSOR
#define CBFPCSEL_SENSOR

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <functional>


class Sensor
{
public:
    Sensor(const YAML::Node& node, std::function<void()> cb);

    void camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg);
    void imgCb(const sensor_msgs::ImageConstPtr& msg);

    bool isInit() {return _tf_init;};
    void allocatePointsBins();
    sensor_msgs::PointCloud2 pcInBody();

    std::vector<std::vector<std::vector<float>>> _bins;
    sensor_msgs::PointCloud2 _points;

    bool _proj_init = false;
    bool _tf_init = false;

    std::function<void()> _notify_node_cb;  // callback to notify node after processing message

    geometry_msgs::TransformStamped _T_cam_body;  // ROS transform from camera frame to body frame

    std::string _topic;  // sensor data topic
    std::string _frame;  // ROS frame of sensor data

    int _bin_h;  // nb of height binning of downscaled image
    int _bin_w;  // nb of width binning of downscaled image [pixels]

    float _percentile;  // percentile filtering for each bin
    int _min_per_bin;  // minimum nb of valid pixels for bins to be considered

    bool _use_camInfo;  // use cameraInfo topic or autocompute intrinsics from resolution and fov
    std::string _camInfo_topic;  // sensor cameraInfo topic

    bool _isPointcloud;  // input is pointcloud instead of image (not used ATM)
    bool _isPolar;  // polar or pinhole projection

    float _pix_to_meters;  // convertion ratio from pixel values to meters

    float _hfov;  // halved horizontal FoV [rad]
    float _vfov;  // halved vertical FoV [rad]
    int _h;  // image height [pixels]
    int _w;  // image width [pixels]

    float _fy, _fx, _cx, _cy;  // intrinsics for pinhole model projection

    float _min_range, _max_range;  // image distance clipping

    int _pix_per_bin;  // nb of pixels per bin for reserve

    ros::Subscriber _camInfoSub;
    ros::Subscriber _imgSub;
};

#endif // CBFPCSEL_SENSOR
