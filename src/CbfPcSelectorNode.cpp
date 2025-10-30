// #include <rclcpp_components/register_node_macro.hpp>

#include "cbf_pc_selector/CbfPcSelectorNode.hpp"
#include "cbf_pc_selector/Sensor.hpp"

#include <string>
#include <yaml-cpp/yaml.h>


CbfPcSelectorNode::CbfPcSelectorNode(const rclcpp::NodeOptions& options)
:   Node("cbf_pc_selector", options),
    _tf_buffer(this->get_clock())
{
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(_tf_buffer, this, true);

    // get yaml cfg
    this->declare_parameter<std::string>("config", "");
    std::string _yaml_path = this->get_parameter("config").as_string();

    RCLCPP_INFO(this->get_logger(), "loaded config: %s", _yaml_path.c_str());

    YAML::Node config = YAML::LoadFile(_yaml_path);

    if (!config["sensors"] || !config["frame_body"]) {
        RCLCPP_ERROR(this->get_logger(), "invalid yaml");
        return;
    }

    for (const auto& node : config["sensors"])
    {
        auto sensor = std::make_shared<Sensor>(node, std::bind(&CbfPcSelectorNode::onSensorCb, this));
        _sensors.push_back(sensor);
    }

    _frame_body = config["frame_body"].as<std::string>();
    rclcpp::Rate rate{1};
    rate.sleep();
    while (!allSensorsAreInit())
    {
        for (auto& sensor : _sensors)
        {
            // register camInfo callback if necessary
            if (sensor->_use_camInfo)
            {
                sensor->_camInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    sensor->_camInfo_topic,
                    rclcpp::SensorDataQoS(),
                    std::bind(&Sensor::camInfoCb, sensor.get(), std::placeholders::_1)
                );
                sensor->_use_camInfo = false;  // prevent re-registration
                RCLCPP_INFO(this->get_logger(),
                    "callback registered to topic %s",
                    sensor->_camInfo_topic.c_str()
                );
            }

            // lookup tf
            if (!sensor->_tf_init)
            {
                try
                {
                    sensor->_T_cam_body = _tf_buffer.lookupTransform(
                        _frame_body,        // target
                        sensor->_frame,     // source
                        tf2::TimePointZero  // latest
                    );
                    sensor->_tf_init = true;
                    RCLCPP_INFO(this->get_logger(),
                        "got sensor-body transform for %s",
                        sensor->_frame.c_str()
                    );
                }
                catch (const tf2::TransformException & ex)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "could not get transform %s - %s: %s",
                        _frame_body.c_str(),
                        sensor->_frame.c_str(),
                        ex.what());
                }
            }
        }
        rate.sleep();
    }

    // get mavros transform is specified
    _publish_mavros = static_cast<bool>(config["frame_mavros"]);
    if (_publish_mavros)
        _frame_mavros = config["frame_mavros"].as<std::string>();

    bool got_transform_mavros = !_publish_mavros;
    while (!got_transform_mavros)
    {
        try
        {
            _T_body_mavros = _tf_buffer.lookupTransform(
                _frame_mavros,      // target
                _frame_body,        // source
                tf2::TimePointZero  // latest

            );
            got_transform_mavros = true;
            RCLCPP_INFO(this->get_logger(), "got body-mavros transform");
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "could not get transform: %s", ex.what());
            rate.sleep();
        }
    }

    // publishers
    _pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cbf_pc_selector/output_pc",
        rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    );
    if (_publish_mavros)
        _mavros_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/cbf_pc_selector/output_mavros",
            rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        );

    // init output pc
    _out_msg.header.frame_id = _frame_body;
    sensor_msgs::PointCloud2Modifier pc_mod(_out_msg);
    pc_mod.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "range", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    // pre-allocate
    size_t nb_pts = 0;
    for (const auto& sensor : _sensors)
        nb_pts += sensor->nbBins();
    pc_mod.reserve(nb_pts);
    pc_mod.resize(0);

    // subscribe to all image topics
    for (const auto& sensor : _sensors)
    {
        if (sensor->_isPointcloud)
        {
            sensor->_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                sensor->_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&Sensor::pcCb, sensor.get(), std::placeholders::_1)
            );
        } else
        {
            sensor->_imgSub = this->create_subscription<sensor_msgs::msg::Image>(
                sensor->_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&Sensor::imgCb, sensor.get(), std::placeholders::_1)
            );
        }

      RCLCPP_INFO(this->get_logger(),
          "callback registered to topic %s",
          sensor->_topic.c_str()
      );
    }
}

bool CbfPcSelectorNode::allSensorsAreInit()
{
    for (auto& sensor : _sensors)
        if (!sensor->_tf_init)
            return false;
    return true;
}


void CbfPcSelectorNode::onSensorCb()
{
    // TODO lock mutex?
    // populate pc message
    _out_msg.header.stamp = this->now();
    sensor_msgs::PointCloud2Modifier pc_mod(_out_msg);

    pc_mod.resize(0);
    for (const auto& sensor : _sensors)
    {
        if (!sensor->hasPoints())
            continue;

        sensor_msgs::msg::PointCloud2 pc_k = sensor->pcInBody();

        _out_msg.data.insert(_out_msg.data.end(), pc_k.data.begin(), pc_k.data.end());
        _out_msg.width += pc_k.width;
        _out_msg.row_step = _out_msg.width * _out_msg.point_step;
    }
    pc_mod.resize(_out_msg.width);
    _pc_pub->publish(_out_msg);

    if (_publish_mavros)
    {
        sensor_msgs::msg::PointCloud2 pc_msg_mavros;
        tf2::doTransform(_out_msg, pc_msg_mavros, _T_body_mavros);
        pc_msg_mavros.header.frame_id = _frame_mavros;
        pc_msg_mavros.header.stamp = _out_msg.header.stamp;
        _mavros_pub->publish(pc_msg_mavros);
    }
}

// RCLCPP_COMPONENTS_REGISTER_NODE(CbfPcSelectorNode)

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CbfPcSelectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
