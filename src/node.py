#! /usr/bin/python3
import numpy as np
import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import WaypointList, Waypoint
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs import point_cloud2
import ros_numpy
import time


class CallbackManager:
    def __init__(self):
        self.pub_ds_pc = None
        self.pub_selected = None
        self.pub_mavros = None

        ## hardcoded stuff yoohoooo\
        self.frame_name = rospy.get_param('/cbf_pc_selector/body_frame')
        self.max_obs = rospy.get_param('/cbf_pc_selector/nb_obstacles')
        self.hfov = np.deg2rad(rospy.get_param('/cbf_pc_selector/hfov') // 2)
        self.vfov = np.deg2rad(rospy.get_param('/cbf_pc_selector/vfov') // 2)
        self.dmin = rospy.get_param('/cbf_pc_selector/min_range')
        self.dmax = rospy.get_param('/cbf_pc_selector/max_range')
        self.shape_img = rospy.get_param('/cbf_pc_selector/bins_v'), rospy.get_param('/cbf_pc_selector/bins_h')

        ## range to pc conversion stuff
        tan_hfov = np.tan(self.hfov)
        tan_vfov = np.tan(self.vfov)
        u = np.arange(0, self.shape_img[1], 1)
        v = np.arange(0, self.shape_img[0], 1)
        u, v = np.meshgrid(u, v, indexing='xy')
        x = np.ones_like(u)
        y = tan_hfov * (1 - u / (u.shape[1] // 2))
        z = tan_vfov * (1 - v / (u.shape[0] // 2))
        self.p = np.stack([x, y, z], axis=0)
        self.p = self.p / np.linalg.norm(self.p, axis=0)

    def cb(self, msg):
        # tic = time.time()

        ## read msg (fast but depend on ros_numpy
        pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        pc = pc[:, [2,1,0]] ; pc[:,2] = - pc[:,2] # rotate to correct frame (TODO not hardcode this)

        ## read msg (slow)
        # pc = np.array([
        #     [z, y, -x]  # with rotation to correct frame
        #     for x, y, z in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        # ])

        ## compute range image
        range = np.linalg.norm(pc, axis=1)
        mask = (range > self.dmin) * (range < self.dmax)
        pc = pc[mask]
        range = range[mask]
        azimuth = np.arctan2(pc[:,1], pc[:,0])
        elevation = np.arcsin(pc[:,2] / range)

        u = np.round(0.5 * (-azimuth + self.hfov) / self.hfov * (self.shape_img[1] - 1)).astype(np.int32)
        v = np.round(0.5 * (-elevation + self.vfov) / self.vfov * (self.shape_img[0] - 1)).astype(np.int32)

        min_pooled = np.ones(self.shape_img) * self.dmax
        min_pooled[v, u] = np.minimum(min_pooled[v, u], range)

        ## select closest points
        nb_obs = min(self.max_obs, sum(min_pooled.flatten() < self.dmax))
        idx = min_pooled.argsort(axis=None)[:nb_obs]
        points = (self.p * min_pooled).reshape(3, -1).T

        ## publish pc
        norm = np.linalg.norm(points, axis=1)
        mask = norm < self.dmax * 0.99
        self.pub_ds_pc.publish(self.pc_to_msg(msg.header.stamp, points[mask], 0, norm[mask]))
        self.pub_selected.publish(self.pc_to_msg(msg.header.stamp, points[idx], norm[idx]))

        mavros_pc_list = WaypointList(0, [Waypoint() for _ in points[idx]])
        for i, id in enumerate(idx):
            mavros_pc_list.waypoints[i].x_lat = points[id,0]
            mavros_pc_list.waypoints[i].y_long = - points[id,1]  # flip to NED
            mavros_pc_list.waypoints[i].z_alt = - points[id,2]  # flip to NED

        self.pub_mavros.publish(mavros_pc_list)

        # toc = time.time()
        # rospy.logwarn(f'{toc-tic}')

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

    manager = CallbackManager()
    manager.pub_ds_pc = rospy.Publisher('/cbf_select/downsamped_pc', PointCloud2, queue_size=1)
    manager.pub_selected = rospy.Publisher('/cbf_select/selected_pc', PointCloud2, queue_size=1)
    manager.pub_mavros = rospy.Publisher('/cbf_select/selected_pc_mavros', WaypointList, queue_size=1)

    rospy.Subscriber('/cbf_select/input', PointCloud2, manager.cb, queue_size=1)

    ## loop
    rospy.spin()
