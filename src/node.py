#! /usr/bin/python3
import numpy as np
import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import WaypointList, Waypoint
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
import time


class CallbackManager:
    def __init__(self):
        self.pub_ds_pc = None
        self.pub_selected = None
        self.pub_mavros = None

        ## hardcoded stuff yoohoooo
        width = 180
        height = 240
        hfov = 86 // 2  # [deg]
        vfov = 106 // 2  # [deg]
        self.pooling_size = 12, 6  # check it divides both width and height
        self.max_obs = 200
        self.frame_name = 'imu'

        ## depth to range conversion stuff
        u = np.arange(0, width, 1)
        v = np.arange(0, height, 1)
        u, v = np.meshgrid(u, v, indexing='xy')
        tan_hfov = np.tan(np.deg2rad(hfov))
        tan_vfov = np.tan(np.deg2rad(vfov))
        self.yz_sqrt = np.sqrt(1 + (tan_hfov * (1 - 2 * u / width))**2 + (tan_vfov * (1 - 2 * v / height))**2)

        ## range to pc conversion stuff
        u = np.arange(0, width // self.pooling_size[1], 1)
        v = np.arange(0, height // self.pooling_size[0], 1)
        u, v = np.meshgrid(u, v, indexing='xy')
        x = np.ones_like(u)
        y = tan_hfov * (1 - u / (u.shape[1] // 2))
        z = tan_vfov * (1 - v / (u.shape[0] // 2))
        self.p = np.stack([x, y, z], axis=0)
        self.p = self.p / np.linalg.norm(self.p, axis=0)

    def cb(self, msg):
        tic = time.time()
        ## read img
        depth_img = np.ndarray((msg.height, msg.width), np.uint8, msg.data, 0).astype(np.float32) / 255 * 5
        range_img = depth_img * self.yz_sqrt
        range_img[range_img <= 0.1] = 1000

        ## minpool
        min_pooled = np.min(range_img.reshape(range_img.shape[0] // self.pooling_size[0], self.pooling_size[0], -1, self.pooling_size[1]), axis=(1, 3))

        ## select closest points
        nb_obs = min(self.max_obs, sum(min_pooled.flatten() < 1000))
        idx = min_pooled.argsort(axis=None)[:nb_obs]

        points = (self.p * min_pooled).reshape(3, -1).T

        ## publish pc
        norm = np.linalg.norm(points, axis=1)
        mask = norm < 500
        self.pub_ds_pc.publish(self.pc_to_msg(msg.header.stamp, points[mask], 0, norm[mask]))
        self.pub_selected.publish(self.pc_to_msg(msg.header.stamp, points[idx], norm[idx]))

        mavros_pc_list = WaypointList(0, [Waypoint() for _ in points[idx]])
        for i, id in enumerate(idx):
            mavros_pc_list.waypoints[i].x_lat = points[id,0]
            mavros_pc_list.waypoints[i].y_long = - points[id,1]  # flip to NED
            mavros_pc_list.waypoints[i].z_alt = - points[id,2]  # flip to NED

        # toc = time.time()
        # print(toc-tic)

    def pc_to_msg(self, ts, pc, norm=0.0, val=0.0):
        data = np.zeros([pc.shape[0], 5], dtype=np.float32)
        data[:, :3] = pc
        data[:, 3] = norm
        data[:, 4] = val
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(stamp=ts, frame_id=self.frame_name)
        return point_cloud2.create_cloud(header, fields, data)


if __name__ == '__main__':
    ## init node
    rospy.init_node('cbf_pc_selector')

    cb_main = CallbackManager()
    cb_main.pub_ds_pc = rospy.Publisher('/cbf_select/downsamped_pc', PointCloud2, queue_size=1)
    cb_main.pub_selected = rospy.Publisher('/cbf_select/selected_pc', PointCloud2, queue_size=1)
    cb_main.pub_mavros = rospy.Publisher('/cbf_select/selected_pc_mavros', WaypointList, queue_size=1)

    pc_sub = rospy.Subscriber('/cbf_select/img_input', Image, cb_main.cb, queue_size=1)

    ## loop
    rospy.spin()
