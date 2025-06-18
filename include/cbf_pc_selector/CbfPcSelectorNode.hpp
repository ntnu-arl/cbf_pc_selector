#ifndef CBFPCSEL
#define CBFPCSEL

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <vector>
#include <memory>

#include "cbf_pc_selector/Sensor.hpp"


class CbfPcSelectorNode
{
public:
    CbfPcSelectorNode();

private:
    bool allSensorsAreInit();
    void onSensorCb();

    sensor_msgs::PointCloud2 _out_msg;

    ros::NodeHandle _nh;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener{_tf_buffer};

    std::vector<std::shared_ptr<Sensor>> _sensors;

    std::string _frame_body;

    std::string _frame_mavros;
    bool _publish_mavros = false;
    geometry_msgs::TransformStamped _T_body_mavros;

    ros::Publisher _pc_pub;
    ros::Publisher _mavros_pub;
};


#endif // CBFPCSEL
