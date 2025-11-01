// #include <rclcpp_components/register_node_macro.hpp>

#include "cbf_pc_selector/CbfPcSelectorNode.hpp"
#include "cbf_pc_selector/Sensor.hpp"

#include <string>
#include <cmath>


static float deg2rad(int deg)
{
    return static_cast<float>(deg) * M_PI / 180;
}

static float deg2rad(double deg)
{
    return static_cast<float>(deg) * M_PI / 180;
}


CbfPcSelectorNode::CbfPcSelectorNode(const rclcpp::NodeOptions& options)
:   Node("cbf_pc_selector", options),
    _tf_buffer(this->get_clock())
{
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(_tf_buffer, this, true);

    // get yaml cfg
    this->get_parameter("frame_body", _frame_body);
    this->get_parameter("frame_mavros", _frame_mavros);
    _publish_mavros = !_frame_mavros.empty();

    // init sensor configs
    std::map<int, SensorConfig> cfgs;

    const std::string prefix = "sensors";
    auto sensor_list = this->list_parameters({prefix}, 5);

    for (const auto& full_param_path : sensor_list.names)
    {
        rclcpp::Parameter p;
        if (!this->get_parameter(full_param_path, p)) continue;

        // full_param_path: "sensors.0.topic" etc
        const std::string head = prefix + ".";
        if (full_param_path.rfind(head, 0) != 0) continue;

        // suffix: "0.topic"
        const std::string suffix = full_param_path.substr(head.size());
        const auto dot = suffix.find('.');
        if (dot == std::string::npos) continue;

        int idx = std::stoi(suffix.substr(0, dot));
        auto& cfg = cfgs[idx];

        const std::string key = suffix.substr(dot + 1);

        if (key == "topic") cfg.topic = p.as_string();
        else if (key == "frame") cfg.frame = p.as_string();
        else if (key == "bins_w") cfg.bins_w = static_cast<int>(p.as_int());
        else if (key == "bins_h") cfg.bins_h = static_cast<int>(p.as_int());
        else if (key == "percentile") cfg.percentile = static_cast<float>(p.as_int()/100.f);
        else if (key == "min_per_bin") cfg.min_per_bin = static_cast<int>(p.as_int());
        else if (key == "mm_resolution") cfg.mm_resolution = static_cast<int>(p.as_int());
        else if (key == "min_range") cfg.min_range = static_cast<float>(p.as_double());
        else if (key == "max_range") cfg.max_range = static_cast<float>(p.as_double());
        else if (key == "is_pointcloud") cfg.is_pointcloud = p.as_bool();
        else if (key == "is_polar") cfg.is_polar = p.as_bool();
        else if (key == "cam_info_topic") cfg.cam_info_topic = p.as_string();
        else if (key == "image_w") cfg.image_w = static_cast<int>(p.as_int());
        else if (key == "image_h") cfg.image_h = static_cast<int>(p.as_int());
        else if (key == "hfov") cfg.hfov = deg2rad(p.as_double()) / 2;
        else if (key == "vfov") cfg.vfov = deg2rad(p.as_double()) / 2;
        else if (key == "nb_pts") cfg.nb_pts = static_cast<int>(p.as_int());
        else if (key == "azimuth_range") {
            auto arr = p.as_double_array();
            if (arr.size() >= 2)
                cfg.azimuth_range = {deg2rad(arr[0]), deg2rad(arr[1])};
        }
        else if (key == "elevation_range") {
            auto arr = p.as_double_array();
            if (arr.size() >= 2)
                cfg.elevation_range = {deg2rad(arr[0]), deg2rad(arr[1])};
        }
    }
    for (auto &kv : cfgs)
    {
        _sensors.push_back(std::make_shared<Sensor>(
            kv.second,
            std::bind(&CbfPcSelectorNode::onSensorCb, this)
        ));
    }
    RCLCPP_INFO(get_logger(), "logged sensor config successfully");

    rclcpp::Rate rate{1};
    rate.sleep();
    while (!allSensorsAreInit())
    {
        for (auto& sensor : _sensors)
        {
            // register camInfo callback if necessary
            if (!sensor->_cfg.cam_info_topic.empty())
            {
                sensor->_camInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    sensor->_cfg.cam_info_topic,
                    rclcpp::SensorDataQoS(),
                    std::bind(&Sensor::camInfoCb, sensor.get(), std::placeholders::_1)
                );
                RCLCPP_INFO(this->get_logger(),
                    "callback registered to topic %s",
                    sensor->_cfg.cam_info_topic.c_str()
                );
                sensor->_cfg.cam_info_topic = "";  // hack to prevent re-registration
            }

            // lookup tf
            if (!sensor->_tf_init)
            {
                try
                {
                    sensor->_T_cam_body = _tf_buffer.lookupTransform(
                        _frame_body,        // target
                        sensor->_cfg.frame,     // source
                        tf2::TimePointZero  // latest
                    );
                    sensor->_tf_init = true;
                    RCLCPP_INFO(this->get_logger(),
                        "got sensor-body transform for %s",
                        sensor->_cfg.frame.c_str()
                    );
                }
                catch (const tf2::TransformException & ex)
                {
                    RCLCPP_WARN(this->get_logger(),
                        "could not get transform %s - %s: %s",
                        _frame_body.c_str(),
                        sensor->_cfg.frame.c_str(),
                        ex.what());
                }
            }
        }
        rate.sleep();
    }

    // get mavros transform is specified
    bool got_transform_mavros = false;
    while (!_frame_mavros.empty() && !got_transform_mavros)
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
        "~/output_pc",
        rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    );
    if (!_frame_mavros.empty())
        _mavros_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/output_mavros",
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
        if (sensor->_cfg.is_pointcloud)
        {
            sensor->_pcSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                sensor->_cfg.topic,
                rclcpp::SensorDataQoS(),
                std::bind(&Sensor::pcCb, sensor.get(), std::placeholders::_1)
            );
        } else
        {
            sensor->_imgSub = this->create_subscription<sensor_msgs::msg::Image>(
                sensor->_cfg.topic,
                rclcpp::SensorDataQoS(),
                std::bind(&Sensor::imgCb, sensor.get(), std::placeholders::_1)
            );
        }

      RCLCPP_INFO(this->get_logger(),
          "callback registered to topic %s",
          sensor->_cfg.topic.c_str()
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

    if (!_frame_mavros.empty())
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
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CbfPcSelectorNode>(opts);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
