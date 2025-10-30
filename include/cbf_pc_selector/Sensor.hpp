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
#include <functional>


struct SensorConfig {
    // required
    std::string topic;  // sensor data topic
    std::string frame;  // ROS frame of sensor data
    int bins_w = 0, bins_h = 0;  // nb of bins of downscaled image [pixels]
    float min_range = 0.f, max_range = 0.f;  // image distance clipping [m]

    // optional / defaults mirror current code
    bool is_pointcloud = false;
    bool is_polar = false;
    int mm_resolution = 1;  // mm per pixel unit
    int min_per_bin = 0;
    float percentile = 0.f;  // [0-1]

    // image cfg
    std::string cam_info_topic = "";  // if not empty -> use camInfo
    int image_w = 0, image_h = 0;
    float hfov = 0.f, vfov = 0.f; // halved FoV [rad]
    // pointcloud cfg
    int nb_pts = 0;
    std::array<float,2> azimuth_range{0.0, 0.0}, elevation_range{0.0, 0.0}; // min and max azimuth and elevation [rad]
};


class Sensor
{
public:
    Sensor(const SensorConfig& cfg, std::function<void()> cb);

    void camInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
    void imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void pcCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    bool hasPoints() { return (bool)_points.width; };
    int nbBins() { return _cfg.bins_h * _cfg.bins_w; };
    sensor_msgs::msg::PointCloud2 pcInBody();

    SensorConfig _cfg;

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

    float _pix_to_meters;  // convertion ratio from pixel values to meters
    int _pix_per_bin;  // nb of pixels per bin for reserve
    float _fy, _fx, _cx, _cy;  // intrinsics for pinhole model projection
    float _az_min, _az_max, _el_min, _el_max;  // min and max azimuth and elevation [rad]
};

#endif // CBFPCSEL_SENSOR
