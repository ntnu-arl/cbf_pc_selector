#include "cbf_pc_selector/Sensor.hpp"

#include <cmath>
#include <algorithm>


Sensor::Sensor(const SensorConfig& cfg,
               std::function<void()> cb)
    : _notify_node_cb(cb)
{
    _cfg = cfg;

    _points.header.frame_id = _cfg.frame;
    _points.height = 1;
    _points.is_dense = false;

    _pix_to_meters = (float)_cfg.mm_resolution / 1000.f;

    if (_cfg.cam_info_topic.empty())
    {
        float bh = static_cast<float>(_cfg.bins_h);
        float bw = static_cast<float>(_cfg.bins_w);
        // compute intrinsics for pinhole projection
        if (!_cfg.is_polar)
        {
            // get resolution and fov info from yaml
            float h = static_cast<float>(_cfg.image_h);
            float w = static_cast<float>(_cfg.image_w);
            _pix_per_bin = std::ceil(h / bh) * std::ceil(w / bw);

            _az_min = -_cfg.hfov / 2;
            _az_max = _cfg.hfov / 2;
            _el_min = -_cfg.vfov / 2;
            _el_max = _cfg.vfov / 2;

            float scale_x = bh / h;
            float scale_y = bw / w;
            _fx = w / 2.0f / std::tan(_cfg.hfov) * scale_x;
            _fy = h / 2.0f / std::tan(_cfg.vfov) * scale_y;
            _cx = w / 2.0f * scale_x;
            _cy = h / 2.0f * scale_y;
        }
        // get angular range for polar projection
        else
        {
            _pix_per_bin = std::ceil(static_cast<float>(_cfg.nb_pts) / bh / bw);

            _az_min = _cfg.azimuth_range[0];
            _az_max = _cfg.azimuth_range[1];
            _el_min = _cfg.elevation_range[0];
            _el_max = _cfg.elevation_range[1];
        }
        allocatePointsBins();
        _proj_init = true;
    }
}


void Sensor::allocatePointsBins()
{
    // allocate bins
    _bins = std::vector<std::vector<std::vector<float>>>(_cfg.bins_h, std::vector<std::vector<float>>(_cfg.bins_w));
    for (size_t u = 0; u < _cfg.bins_h; u++)
        for (size_t v = 0; v < _cfg.bins_w; v++)
            _bins[u][v].reserve(_pix_per_bin);

    // allocate pc2
    sensor_msgs::PointCloud2Modifier pc_mod(_points);
    pc_mod.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "range", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    pc_mod.reserve(_cfg.bins_h * _cfg.bins_w);
    pc_mod.resize(0);
}


void Sensor::camInfoCb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
    if (!_proj_init)
    {
        float h = static_cast<float>(msg->height);
        float w = static_cast<float>(msg->width);
        float bh = static_cast<float>(_cfg.bins_h);
        float bw = static_cast<float>(_cfg.bins_w);

        _pix_per_bin = std::ceil(h / bh) * std::ceil(w / bw);

        float scale_x = bh / h;
        float scale_y = bw / w;
        _fx = static_cast<float>(msg->k[0]) * scale_x;
        _fy = static_cast<float>(msg->k[4]) * scale_y;
        _cx = static_cast<float>(msg->k[2]) * scale_x;
        _cy = static_cast<float>(msg->k[5]) * scale_y;

        allocatePointsBins();
        _proj_init = true;
    }
}


void Sensor::imgCb(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    // skip if cameraInfo not received
    if (!_proj_init)
    {
        auto logger = rclcpp::get_logger("cbf_cp_selector").get_child("sensor");
        RCLCPP_WARN(logger, "waiting for cameraInfo on %s", _cfg.cam_info_topic.c_str());
        return;
    }

    // loop over all pixels
    size_t width = msg->width;
    size_t height = msg->height;
    size_t step = msg->step;
    const uint8_t* data = msg->data.data();

    int u_bin, v_bin;
    for (size_t u = 0; u < height; ++u)
    {
        if (msg->encoding == "32FC1")
        {
            const float* row = reinterpret_cast<const float*>(data + u * step);
            for (size_t v = 0; v < width; ++v)
            {
                float distance = row[v] * _pix_to_meters;

                // disregard invalid pixels
                if (distance < _cfg.min_range || distance > _cfg.max_range)
                    continue;

                // store in corresponding bin
                v_bin = (float)v / width * _cfg.bins_w;
                u_bin = (float)u / height * _cfg.bins_h;
                v_bin = std::min(std::max(0, v_bin), _cfg.bins_w - 1);
                u_bin = std::min(std::max(0, u_bin), _cfg.bins_h - 1);
                _bins[u_bin][v_bin].push_back(distance);
            }
        }
        else if (msg->encoding == "16UC1" || msg->encoding == "mono16")
        {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(data + u * step);
            for (size_t v = 0; v < width; ++v)
            {
                float distance = static_cast<float>(row[v]) * _pix_to_meters;

                // disregard invalid pixels
                if (distance < _cfg.min_range || distance > _cfg.max_range)
                    continue;

                // store in corresponding bin
                v_bin =static_cast<float>(v) / width * _cfg.bins_w;
                u_bin =static_cast<float>(u) / height * _cfg.bins_h;
                v_bin = std::min(std::max(0, v_bin), _cfg.bins_w - 1);
                u_bin = std::min(std::max(0, u_bin), _cfg.bins_h - 1);
                _bins[u_bin][v_bin].push_back(distance);
            }
        }
        else
        {
            auto logger = rclcpp::get_logger("cbf_cp_selector").get_child("sensor");
            RCLCPP_WARN(logger, "image encoding not supported");
            return;
        }
    }

    // create points for each pixels
    binsToPoints();
    _points.header.stamp = msg->header.stamp;

    // notify main node to publish
    _notify_node_cb();
}


void Sensor::pcCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    int u_bin, v_bin;

    for (sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x"), y_it(*msg, "y"), z_it(*msg, "z");
         x_it != x_it.end();
         ++x_it, ++y_it, ++z_it)
    {
        const float x = *x_it;
        const float y = *y_it;
        const float z = *z_it;

        float range = std::sqrt(x * x + y * y + z * z);

        // disregard invalid pixels
        if (range < _cfg.min_range || range > _cfg.max_range)
            continue;

        float az = atan2(y, x);
        float el = atan2(z, std::sqrt(x * x + y * y));

        v_bin = static_cast<int>(std::floor((az - _az_min) / (_az_max - _az_min) * _cfg.bins_w));
        u_bin = static_cast<int>(std::floor((el - _el_min) / (_el_max - _el_min) * _cfg.bins_h));
        v_bin = std::min(std::max(0, v_bin), _cfg.bins_w - 1);
        u_bin = std::min(std::max(0, u_bin), _cfg.bins_h - 1);
        _bins[u_bin][v_bin].push_back(range);
    }

    // create points for each pixels
    binsToPoints();
    _points.header.stamp = msg->header.stamp;

    // notify main node to publish
    _notify_node_cb();
}


void Sensor::binsToPoints()
{
    sensor_msgs::PointCloud2Modifier pc_mod(_points);
    size_t nb_points = 0;
    for (size_t u = 0; u < _cfg.bins_h; ++u)
        for (size_t v = 0; v < _cfg.bins_w; ++v)
            if (_bins[u][v].size() > _cfg.min_per_bin)
                ++nb_points;
    pc_mod.resize(nb_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(_points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(_points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(_points, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_r(_points, "range");

    for (size_t u = 0; u < _cfg.bins_h; ++u)
    {
        for (size_t v = 0; v < _cfg.bins_w; ++v)
        {
            if (_bins[u][v].size() > _cfg.min_per_bin)
            {
                // parial sort
                size_t idx = static_cast<size_t>(floor(_bins[u][v].size() * _cfg.percentile));
                std::nth_element(_bins[u][v].begin(), _bins[u][v].begin() + idx, _bins[u][v].end());

                // point coordinates
                if (_cfg.is_polar)
                {
                    float range = _bins[u][v][idx];
                    float azimuth = _az_min + (static_cast<float>(v) + 0.5f) * (_az_max - _az_min) / (_cfg.bins_w);
                    float elevation = _el_min + (static_cast<float>(u) + 0.5f) * (_el_max - _el_min) / (_cfg.bins_h);

                    *iter_x = range * std::cos(elevation) * std::cos(azimuth);
                    *iter_y = range * std::cos(elevation) * std::sin(azimuth);
                    *iter_z = range * std::sin(elevation);
                    *iter_r = range;
        		}
                else
                {
                    *iter_z = _bins[u][v][idx];
                    *iter_x = (static_cast<float>(v) + 0.5f - _cx) / _fx * _bins[u][v][idx];
                    *iter_y = (static_cast<float>(u) + 0.5f - _cy) / _fy * _bins[u][v][idx];

                    float x = *iter_x;
                    float y = *iter_y;
                    float z = *iter_z;
                    *iter_r = std::sqrt(x * x + y * y + z * z);
                }

                ++iter_x; ++iter_y; ++iter_z, ++iter_r;
                ++nb_points;
            }
            _bins[u][v].clear();
        }
    }
}


sensor_msgs::msg::PointCloud2 Sensor::pcInBody()
{
    sensor_msgs::msg::PointCloud2 pc_body;
    tf2::doTransform(_points, pc_body, _T_cam_body);
    return pc_body;
}
