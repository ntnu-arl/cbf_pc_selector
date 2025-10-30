#ifndef CBFPCSEL
#define CBFPCSEL

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>

#include "cbf_pc_selector/Sensor.hpp"


class CbfPcSelectorNode : public rclcpp::Node
{
public:
    explicit CbfPcSelectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    bool allSensorsAreInit();
    void onSensorCb();

    sensor_msgs::msg::PointCloud2 _out_msg;

    tf2_ros::Buffer _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    std::vector<std::shared_ptr<Sensor>> _sensors;

    std::string _frame_body;
    std::string _frame_mavros;
    bool _publish_mavros = false;
    geometry_msgs::msg::TransformStamped _T_body_mavros;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pc_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _mavros_pub;
};


#endif // CBFPCSEL
