#include "cbf_pc_selector/Sensor.hpp"

#include <stdexcept>
#include <cmath>
#include <algorithm>


Sensor::Sensor(const YAML::Node& node,
               std::function<void()> cb)
    : _notify_node_cb(cb)
{
    _topic = node["topic"].as<std::string>();
    _frame = node["frame"].as<std::string>();
    _points.header.frame_id = _frame;

    _isPointcloud = node["is_pointcloud"] ? node["is_pointcloud"].as<bool>() : false;
    _isPolar = node["is_polar"] ? node["is_polar"].as<bool>() : false;

    int mm_res = node["mm_resolution"] ? node["mm_resolution"].as<int>() : 1;
    _pix_to_meters = (float)mm_res / 1000.f;

    _min_range = node["min_range"].as<float>();
    _max_range = node["max_range"].as<float>();

    if (!node["bins_h"] || !node["bins_w"])
        throw std::runtime_error("yaml missing bins");
    _bin_h = node["bins_h"].as<int>();
    _bin_w = node["bins_w"].as<int>();

    _percentile = node["percentile"] ? node["percentile"].as<float>() / 100 : 0;
    float min_ratio = node["min_ratio_per_bin"] ? node["min_ratio_per_bin"].as<float>() : 0.f;
    _min_per_bin = (int) ceil(min_ratio * _bin_h * _bin_w);

    if (node["cam_info_topic"])
    {
        _use_camInfo = true;
        _camInfo_topic = node["cam_info_topic"].as<std::string>();
    }
    else
    {
        _use_camInfo = false;

        // compute intrinsics for pinhole projection
        if (!_isPolar)
        {
            // get resolution and fov info from yaml
            float h = node["image_h"] ? node["image_h"].as<float>() : 0.f;
            float w = node["image_w"] ? node["image_w"].as<float>() : 0.f;
            _pix_per_bin = std::ceil(h / _bin_h) * std::ceil(w / _bin_w);

            if (!node["hfov"])
                throw std::runtime_error("yaml missing hfov");
            _hfov = node["hfov"].as<float>() * M_PI / 180 / 2;

            if (!node["vfov"])
                throw std::runtime_error("yaml missing vfov");
            _vfov = node["vfov"].as<float>() * M_PI / 180 / 2;

            _az_min = -_hfov / 2;
            _az_max = _hfov / 2;
            _el_min = -_vfov / 2;
            _el_max = _vfov / 2;

            float scale_x = (float)_bin_h / h;
            float scale_y = (float)_bin_w / w;
            _fx = w / 2.0f / std::tan(_hfov) * scale_x;
            _fy = h / 2.0f / std::tan(_vfov) * scale_y;
            _cx = w / 2.0f * scale_x;
            _cy = h / 2.0f * scale_y;
        }
        // get angular range for polar projection
        else
        {
            float nb_pts = node["nb_pts"] ? node["nb_pts"].as<float>() : 0.f;
            _pix_per_bin = std::ceil(nb_pts / _bin_h / _bin_w);

            if (!node["azimuth_range"] || !node["azimuth_range"].IsSequence() || !(node["azimuth_range"].size() == 2))
                throw std::runtime_error("yaml missing azimuth_range");
            if (!node["elevation_range"] || !node["elevation_range"].IsSequence() || !(node["elevation_range"].size() == 2))
                throw std::runtime_error("yaml missing elevation_range");
            _az_min = node["azimuth_range"][0].as<float>() * M_PI / 180;
            _az_max = node["azimuth_range"][1].as<float>() * M_PI / 180;
            _el_min = node["elevation_range"][0].as<float>() * M_PI / 180;
            _el_max = node["elevation_range"][1].as<float>() * M_PI / 180;
        }
        allocatePointsBins();
        _proj_init = true;
    }
}


void Sensor::allocatePointsBins()
{
    // allocate bins
    _bins = std::vector<std::vector<std::vector<float>>>(_bin_h, std::vector<std::vector<float>>(_bin_w));
    for (size_t u = 0; u < _bin_h; u++)
        for (size_t v = 0; v < _bin_w; v++)
            _bins[u][v].reserve(_pix_per_bin);

    // allocate pc2
    sensor_msgs::PointCloud2Modifier pc_mod(_points);
    pc_mod.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32
    );
    pc_mod.reserve(_bin_h * _bin_w);
    pc_mod.resize(0);
}


void Sensor::camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
    if (!_proj_init)
    {
        float h = (float)msg->height;
        float w = (float)msg->width;

        _pix_per_bin = std::ceil(h / _bin_h) * std::ceil(w / _bin_w);

        float scale_x = (float)_bin_h / h;
        float scale_y = (float)_bin_w / w;
        _fx = msg->K[0] * scale_x;
        _fy = msg->K[4] * scale_y;
        _cx = msg->K[2] * scale_x;
        _cy = msg->K[5] * scale_y;

        allocatePointsBins();
        _proj_init = true;
    }
}


void Sensor::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
    // skip if cameraInfo not received
    if (!_proj_init)
    {
        ROS_WARN("[cbf_cp_selector] waiting for cameraInfo on %s", _camInfo_topic.c_str());
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
                if (distance < _min_range || distance > _max_range)
                    continue;

                // store in corresponding bin
                v_bin = (float)v / width * _bin_w;
                u_bin = (float)u / height * _bin_h;
                v_bin = std::min(std::max(0, v_bin), _bin_w - 1);
                u_bin = std::min(std::max(0, u_bin), _bin_h - 1);
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
                if (distance < _min_range || distance > _max_range)
                    continue;

                // store in corresponding bin
                v_bin = (float)v / width * _bin_w;
                u_bin = (float)u / height * _bin_h;
                v_bin = std::min(std::max(0, v_bin), _bin_w - 1);
                u_bin = std::min(std::max(0, u_bin), _bin_h - 1);
                _bins[u_bin][v_bin].push_back(distance);
            }
        }
        else
        {
            ROS_WARN("[cbf_pc_selector] image encoding not supported");
            return;
        }
    }

    // create points for each pixels
    binsToPoints();
    _points.header.stamp = msg->header.stamp;

    // notify main node to publish
    _notify_node_cb();
}


void Sensor::pcCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    int u_bin, v_bin;
    for (sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x"), y_it(*msg, "y"), z_it(*msg, "z");
         x_it != x_it.end();
         ++x_it, ++y_it, ++z_it)
    {
        float x = *x_it;
        float y = *y_it;
        float z = *z_it;

        float range = std::sqrt(x * x + y * y + z * z);

        // disregard invalid pixels
        if (range < _min_range || range > _max_range)
            continue;

        float az = atan2(y, x);
        float el = atan2(z, std::sqrt(x * x + y * y));

        v_bin = static_cast<int>((az - _az_min) / (_az_max - _az_min) * _bin_w);
        u_bin = static_cast<int>((el - _el_min) / (_el_max - _el_min) * _bin_h);
        v_bin = std::min(std::max(0, v_bin), _bin_w - 1);
        u_bin = std::min(std::max(0, u_bin), _bin_h - 1);
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
    sensor_msgs::PointCloud2Iterator<float> iter_x(_points, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(_points, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(_points, "z");

    size_t nb_points = 0;
    for (size_t u = 0; u < _bin_h; ++u)
    {
        for (size_t v = 0; v < _bin_w; ++v)
        {
            if (_bins[u][v].size() > _min_per_bin)
            {
                // parial sort
                size_t idx = static_cast<size_t>(floor(_bins[u][v].size() * _percentile));
                std::nth_element(_bins[u][v].begin(), _bins[u][v].begin() + idx, _bins[u][v].end());

                // point coordinates
                if (_isPolar)
                {
                    float range = _bins[u][v][idx];
                    float azimuth = _az_min + ((float)v + 0.5f) * (_az_max - _az_min) / (_bin_w - 1);
                    float elevation = _el_min + ((float)u + 0.5f) * (_el_max - _el_min) / (_bin_h - 1);

                    *iter_x = range * cos(elevation) * cos(azimuth);
                    *iter_y = range * cos(elevation) * sin(azimuth);
                    *iter_z = range * sin(elevation);
                }
                else
                {
                    *iter_z = _bins[u][v][idx];
                    *iter_x = (v + 0.5 - _cx) / _fx * _bins[u][v][idx];
                    *iter_y = (u + 0.5 - _cy) / _fy * _bins[u][v][idx];
                }

                ++iter_x; ++iter_y; ++iter_z;
                ++nb_points;
            }
            _bins[u][v].clear();
        }
    }
    sensor_msgs::PointCloud2Modifier pc_mod(_points);
    pc_mod.resize(nb_points);
}


sensor_msgs::PointCloud2 Sensor::pcInBody()
{
    sensor_msgs::PointCloud2 pc_body;
    tf2::doTransform(_points, pc_body, _T_cam_body);
    return pc_body;
}
