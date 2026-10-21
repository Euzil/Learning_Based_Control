import yaml
import os
import time as _time
import numpy as np
import threading
import copy
from collections import deque
from dataclasses import dataclass

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from skin_msgs.msg import SkinPatchInfo, SkinPatchData
from geometry_msgs.msg import Pose

from skin_real.scn.sc.data_publisher import DataPublisher
from skin_real.skin_interface import SkinInterface


def parse_skin_config_file(cfg_file: str):
    with open(cfg_file, 'r') as f:
        data = yaml.safe_load(f)
    
    base_frame = data['sp_extrinsics']['base_frame']
    prefix = data['sp_driver_settings']['prefix']
    tf_base_rsc = np.array(data['sp_extrinsics']['tf_base_rsc'])
    tf_sc_local = np.stack([np.array(pose['htf']) for pose in data['sc_patch']['sc_poses']])
    sc_id = [pose['sc_id'] for pose in data['sc_patch']['sc_poses']]
    
    # base frame name, tf to base, cell tfs, cell ids
    return base_frame, prefix, tf_base_rsc, tf_sc_local, sc_id


@dataclass
class PatchConfig:
    skin_package: str
    pub_rate: float = 100.0
    proximity_gain: float = 1.0


class SkinPatch:
    """SkinPatch
    publishes the data of a skin patch by reading from interface 
    """
    
    def __init__(self, 
                 node: Node, 
                 interface: SkinInterface, 
                 name: str, 
                 cfg_file: str, 
                 patch_config: PatchConfig):
        
        self.node = node
        self.cfg = patch_config
        
        # parse the skin configuration file
        (base_frame, 
         prefix,
         tf_base_rsc, 
         tf_sc_local, 
         sc_id) = parse_skin_config_file(cfg_file)
        
        self._base_frame = base_frame
        self._id_to_idx = {id: idx for idx, id in enumerate(sc_id)}
        
        # Per-cell latest raw values from reader thread
        num_cells = len(self._id_to_idx)
        self._num_cells = num_cells
        self._latest_raw = [None] * num_cells
        
        # convert to base frame
        self._poses = np.matmul(tf_base_rsc, tf_sc_local)

        # data create publisher
        self._raw_data_msg = SkinPatchData()
        self._raw_data_msg.prox = [0.0] * num_cells
        self._raw_data_msg.force1 = [0.0] * num_cells
        self._raw_data_msg.force2 = [0.0] * num_cells
        self._raw_data_msg.force3 = [0.0] * num_cells
        self._raw_data_msg.acc_x = [0.0] * num_cells
        self._raw_data_msg.acc_y = [0.0] * num_cells
        self._raw_data_msg.acc_z = [0.0] * num_cells
        self._raw_data_msg.temp = [0.0] * num_cells

        self._raw_data_pub = node.create_publisher(
            SkinPatchData,
            f'{prefix}/{name}/raw',
            1)
        
        # create info publisher
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        
        self._static_info_pub = node.create_publisher(
            SkinPatchInfo, 
            f'{prefix}/{name}/info', 
            qos_profile=latched_qos
        )
        self._publish_static_info()

        self._data_lock = threading.Lock()
        self._pub_timer = node.create_timer(1.0 / self.cfg.pub_rate, self._publish_callback)
        
        # register callback in interface
        interface.add_data_callback(self._data_callback)
        
    def stop(self):
        """Clean up the timer on shutdown safely"""
        try:
            if self._pub_timer and not self._pub_timer.is_canceled():
                self._pub_timer.cancel()
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # callbacks and publishers

    def _data_callback(self, data: DataPublisher.ScData):
        """Fast path on UDP reader thread 
        """
        sc_id, sc_data = data
        idx = self._id_to_idx.get(sc_id)
        if idx is None:
            return
        raw = (float(sc_data[0]), float(sc_data[1]), float(sc_data[2]), float(sc_data[3]),
               float(sc_data[4]), float(sc_data[5]), float(sc_data[6]), float(sc_data[7]))
        with self._data_lock:
            self._latest_raw[idx] = raw

    def _publish_callback(self):
        """Processes raw data → messages and publishes
        """
        with self._data_lock:
            raw_snapshot = list(self._latest_raw)

        for idx in range(self._num_cells): 
            raw = raw_snapshot[idx]
            if raw is None:
                continue

            raw_prox, raw_f1, raw_f2, raw_f3, raw_ax, raw_ay, raw_az, raw_temp = raw

            # --- raw data msg ---
            self._raw_data_msg.prox[idx] = raw_prox
            self._raw_data_msg.force1[idx] = raw_f1
            self._raw_data_msg.force2[idx] = raw_f2
            self._raw_data_msg.force3[idx] = raw_f3
            self._raw_data_msg.acc_x[idx] = raw_ax
            self._raw_data_msg.acc_y[idx] = raw_ay
            self._raw_data_msg.acc_z[idx] = raw_az
            self._raw_data_msg.temp[idx] = raw_temp

        # Publish
        self._raw_data_msg.header.stamp = self.node.get_clock().now().to_msg()
        self._raw_data_pub.publish(self._raw_data_msg)
        
    def _publish_static_info(self):        
        translations = self._poses[:, 0:3, 3]
        rotation_mats = self._poses[:, 0:3, 0:3]
        quats = Rotation.from_matrix(rotation_mats).as_quat()

        msg = SkinPatchInfo()
        msg.header.frame_id = self._base_frame
        for i in range(self._poses.shape[0]):
            pose = Pose()
            pose.position.x = translations[i, 0]
            pose.position.y = translations[i, 1]
            pose.position.z = translations[i, 2]
            pose.orientation.x = quats[i, 0]
            pose.orientation.y = quats[i, 1]
            pose.orientation.z = quats[i, 2]
            pose.orientation.w = quats[i, 3]        
            msg.poses.append(pose)
        self._static_info_pub.publish(msg)