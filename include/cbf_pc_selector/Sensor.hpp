#ifndef CBFPCSEL_SENSOR
#define CBFPCSEL_SENSOR

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <functional>


class Sensor
{
public:
    Sensor(const YAML::Node& node, std::function<void()> cb);

    void camInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void pcCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    bool hasPoints() { return (bool)_points.width; };
    size_t nbBins() { return _bin_h * _bin_w; };
    sensor_msgs::msg::PointCloud2 pcInBody();

    // TODO make the field below accessible via getters?
    std::string _topic;  // sensor data topic
    std::string _frame;  // ROS frame of sensor data

    std::string _camInfo_topic;  // sensor cameraInfo topic
    bool _use_camInfo;  // use cameraInfo topic or autocompute intrinsics from resolution and fov

    bool _isPointcloud;  // input is pointcloud instead of image (not used ATM)
    bool _isPolar;  // polar or pinhole projection
    bool _proj_init = false;
    bool _tf_init = false;

    geometry_msgs::msg::TransformStamped _T_cam_body;  // ROS transform from camera frame to body frame
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camInfoSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pcSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _imgSub;
private:
    void allocatePointsBins();
    void binsToPoints();

    std::vector<std::vector<std::vector<float>>> _bins;
    sensor_msgs::msg::PointCloud2 _points;

    std::function<void()> _notify_node_cb;  // callback to notify node after processing message

    int _bin_h;  // nb of height binning of downscaled image
    int _bin_w;  // nb of width binning of downscaled image [pixels]
    float _pix_to_meters;  // convertion ratio from pixel values to meters

    float _percentile;  // percentile filtering for each bin
    int _min_per_bin;  // minimum nb of valid pixels for bins to be considered
    int _pix_per_bin;  // nb of pixels per bin for reserve

    float _hfov, _vfov;  // halved horizontal and vertical FoV [rad]
    float _az_min, _az_max, _el_min, _el_max;  // min and max azimuth and elevation[rad]

    float _fy, _fx, _cx, _cy;  // intrinsics for pinhole model projection

    float _min_range, _max_range;  // image distance clipping
};

#endif // CBFPCSEL_SENSOR
