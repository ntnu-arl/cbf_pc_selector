#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <vector>
#include <algorithm>

#include <iostream>
#include <chrono>


struct PixelPoint
{
    float x, y, z;
};

bool ppSorter(PixelPoint& p1, PixelPoint& p2)
{
    return p1.z < p2.z;
}


class CbfPcSelector
{
public:
    CbfPcSelector()
    {
        // get params
        _nh.getParam("/cbf_pc_selector/nb_obstacles", _nb_pts_max);
        _nh.getParam("/cbf_pc_selector/min_ratio_per_bin", _min_ratio_bin);
        _nh.getParam("/cbf_pc_selector/percentile", _percentile);
        _percentile = _percentile / 100;
        _nh.getParam("/cbf_pc_selector/min_range", _min_range);
        _min_range = _min_range * 1000;
        _nh.getParam("/cbf_pc_selector/max_range", _max_range);
        _max_range = _max_range * 1000;

        // allocate bins
        _nh.getParam("/cbf_pc_selector/bins_h", _image_width);
        _nh.getParam("/cbf_pc_selector/bins_v", _image_height);
        _points.reserve(_image_width * _image_height);

        _bins = std::vector<std::vector<std::vector<float>>>(_image_height, std::vector<std::vector<float>>(_image_width));
        for (size_t u = 0; u < _image_height; u++)
            for (size_t v = 0; v < _image_width; v++)
                _bins[u][v].reserve(10000);


        // subscribers
        _camera_info_sub = _nh.subscribe("/cbf_pc_selector/camera_info", 1, &CbfPcSelector::cameraInfoCb, this);
        _image_sub = _nh.subscribe("/cbf_pc_selector/input_image", 1, &CbfPcSelector::imageCb, this);
        _pc_pub = _nh.advertise<sensor_msgs::PointCloud2>("/cbf_pc_selector/output_pc", 1);
    }

private:

    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        float scale_x = (float)_image_width / msg->width;
        float scale_y = (float)_image_height / msg->height;
        _min_per_bin = _min_ratio_bin / (scale_x * scale_y);
        _fx = msg->K[0] * scale_x;
        _fy = msg->K[4] * scale_y;
        _cx = msg->K[2] * scale_x;
        _cy = msg->K[5] * scale_y;
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        // exit if intrinsics not received
        if (_fx == 0.f)
        {
            return;
        }

        // auto start = std::chrono::system_clock::now();

        // loop over all pixels
        size_t width = msg->width;
        size_t height = msg->height;
        size_t step = msg->step;
        const uint8_t* data = msg->data.data();

        int u_bin, v_bin;
        for (size_t u = 0; u < height; ++u)
        {
            const uint16_t* row = reinterpret_cast<const uint16_t*>(data + u * step);
            for (size_t v = 0; v < width; ++v)
            {
                uint16_t depth_mm = row[v];

                // disregard invalid pixels
                if (depth_mm < _min_range || depth_mm > _max_range)
                    continue;

                // get corresponding bin
                v_bin = (float)v / width * _image_width;
                v_bin = std::min(std::max(0, v_bin), _image_width - 1);
                u_bin = (float)u / height * _image_height;
                u_bin = std::min(std::max(0, u_bin), _image_height - 1);

                // store
                _bins[u_bin][v_bin].push_back(depth_mm / 1000.f);
            }
        }

        // create points for each pixels
        _points.clear();
        for (size_t u = 0; u < _image_height; u++)
        {
            for (size_t v = 0; v < _image_width; v++)
            {
                if (_bins[u][v].size() > _min_per_bin)
                {
                    // parial sort
                    size_t idx = static_cast<size_t>(_bins[u][v].size() * _percentile);
                    std::nth_element(_bins[u][v].begin(), _bins[u][v].begin() + idx, _bins[u][v].end());

                    PixelPoint pp;
                    pp.z = _bins[u][v][idx];
                    pp.x = (v + 0.5 - _cx) / _fx * pp.z;
                    pp.y = (u + 0.5 - _cy) / _fy * pp.z;
                    _points.push_back(pp);
                }
                _bins[u][v].clear();
            }
        }

        // sort n-th closest points
        size_t nb_points;
        if (_nb_pts_max < _points.size())
        {
            std::nth_element(_points.begin(), _points.begin() + _nb_pts_max, _points.end(), &ppSorter);
            nb_points = _nb_pts_max;
        }
        else
        {
            nb_points = _points.size();
        }

        // fill PointCloud2
        sensor_msgs::PointCloud2 pc_msg;
        _nh.getParam("/cbf_pc_selector/body_frame", pc_msg.header.frame_id);

        sensor_msgs::PointCloud2Modifier pc_modifier(pc_msg);
        pc_modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32
        );
        pc_modifier.resize(nb_points);

        sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");

        for (size_t i = 0; i < nb_points; i++)
        {
            PixelPoint& pp = _points[i];

            // insert in pc
            *iter_x = pp.x;
            *iter_y = pp.y;
            *iter_z = pp.z;

            // increment iterators
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        // publish
        pc_msg.header.stamp = msg->header.stamp;
        _pc_pub.publish(pc_msg);

        // auto end = std::chrono::system_clock::now();
        // std::chrono::duration<double> elapsed_seconds = end-start;
        // static double elapsed_time_sum = 0;
        // static size_t count = 0;
        // elapsed_time_sum += elapsed_seconds.count();
        // count++;

        // if (count % 90 == 0){
        // std::cout << "average elapsed time: " << elapsed_time_sum / (double)count * 1e3 << " ms\n";
        }
    }

    ros::NodeHandle _nh;

    ros::Subscriber _camera_info_sub;
    ros::Subscriber _image_sub;
    ros::Publisher _pc_pub;

    std::vector<std::vector<std::vector<float>>> _bins;
    std::vector<PixelPoint> _points;

    int _image_height, _image_width, _nb_pts_max, _min_per_bin;
    float _fx = 0.f, _fy, _cx, _cy;
    float _min_range, _max_range;
    float _percentile, _min_ratio_bin;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cbf_pc_selector");
    CbfPcSelector selector;
    ros::spin();
    return 0;
}
