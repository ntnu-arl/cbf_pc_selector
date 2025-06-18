#include "cbf_pc_selector/CbfPcSelectorNode.hpp"
#include "cbf_pc_selector/Sensor.hpp"

#include <string>
#include <yaml-cpp/yaml.h>


CbfPcSelectorNode::CbfPcSelectorNode()
{
    // get yaml cfg
    std::string _yaml_path;
    _nh.getParam("/cbf_pc_selector/config_file", _yaml_path);

    YAML::Node config = YAML::LoadFile(_yaml_path);

    if (!config["sensors"] || !config["frame_body"]) {
        ROS_ERROR("[cbf_cp_selector] invalid yaml");
        return;
    }

    for (const auto& node : config["sensors"])
    {
        auto sensor = std::make_shared<Sensor>(node, std::bind(&CbfPcSelectorNode::onSensorCb, this));
        _sensors.push_back(sensor);
    }

    _frame_body = config["frame_body"].as<std::string>();
    while (!allSensorsAreInit())
    {
        for (auto& sensor : _sensors)
        {
            // register camInfo callback if necessary
            if (sensor->_use_camInfo)
            {
                sensor->_camInfoSub = _nh.subscribe<sensor_msgs::CameraInfo>(sensor->_camInfo_topic, 1, &Sensor::camInfoCb, sensor.get());
                sensor->_use_camInfo = false;  // set to false to prevent re-registration
                ROS_INFO("[cbf_pc_selector] callback registered to topic %s", sensor->_camInfo_topic.c_str());
            }

            // lookup tf
            if (!sensor->_tf_init)
            {
                try
                {
                    sensor->_T_cam_body = _tf_buffer.lookupTransform(_frame_body, sensor->_frame, ros::Time(0));
                    sensor->_tf_init = true;
                    ROS_INFO("[cbf_pc_selector] got sensor-body transform for %s", sensor->_frame.c_str());
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN(
                        "[cbf_pc_selector] could not get transform %s - %s: %s",
                        _frame_body.c_str(),
                        sensor->_frame.c_str(),
                        ex.what());
                }
            }
        }
        ros::Duration(1).sleep();
    }

    // get mavros transform is specified
    _publish_mavros = (bool)config["frame_mavros"];
    if (_publish_mavros)
        _frame_mavros = config["frame_mavros"].as<std::string>();

    bool got_transform_mavros = !_publish_mavros;
    while (!got_transform_mavros)
    {
        try
        {
            _T_body_mavros = _tf_buffer.lookupTransform(_frame_mavros, _frame_body, ros::Time(0));
            got_transform_mavros = true;
            ROS_INFO("[cbf_pc_selector] got body-mavros transform");
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("[cbf_pc_selector] could not get transform: %s", ex.what());
            ros::Duration(1).sleep();
        }
    }

    // publishers
    _pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cbf_pc_selector/output_pc", 1);
    if (_publish_mavros)
        _mavros_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cbf_pc_selector/output_mavros", 1);


    // finally subscribe to all image topics
    for (const auto& sensor : _sensors)
    {
        sensor->_imgSub = _nh.subscribe<sensor_msgs::Image>(sensor->_topic, 1, &Sensor::imgCb, sensor.get());
        ROS_INFO("[cbf_pc_selector] callback registered to topic %s", sensor->_topic.c_str());
    }
}

bool CbfPcSelectorNode::allSensorsAreInit()
{
    for (auto& sensor : _sensors)
        if (!sensor->isInit())
            return false;
    return true;
}


void CbfPcSelectorNode::onSensorCb()
{
    // TODO lock mutex?

    // populate pc message
    sensor_msgs::PointCloud2 pc_msg;
    pc_msg.header.frame_id = _frame_body;
    pc_msg.header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2Modifier pc_mod(pc_msg);
    pc_mod.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32
    );
    pc_mod.reserve(100000);

    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

    size_t nb_points = 0;
    for (const auto& sensor : _sensors)
    {
        sensor_msgs::PointCloud2 pc_k = sensor->pcInBody();

        pc_msg.data.insert(pc_msg.data.end(), pc_k.data.begin(), pc_k.data.end());
        pc_msg.width += pc_k.width;
        pc_msg.row_step = pc_msg.width * pc_msg.point_step;
    }
    pc_mod.resize(pc_msg.width);
    _pc_pub.publish(pc_msg);

    if (_publish_mavros)
    {
        sensor_msgs::PointCloud2 pc_msg_mavros;
        tf2::doTransform(pc_msg, pc_msg_mavros, _T_body_mavros);
        pc_msg_mavros.header.frame_id = _frame_mavros;
        pc_msg_mavros.header.stamp = pc_msg.header.stamp;
        _mavros_pub.publish(pc_msg_mavros);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cbf_pc_selector");
    CbfPcSelectorNode node;
    ros::spin();
    return 0;
}
