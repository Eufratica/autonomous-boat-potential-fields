#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hitl_pretrain_pointcloud_node.py

Pré-treino Human-in-the-Loop com SITL + Gazebo + Livox Mid-70.

Agora:
- humano desvia via RC (throttle / yaw)
- CSV ENXUTO:
  t, phase,
  min_front, min_left, min_right,
  speed, yaw,
  dist_to_wp, bearing_to_wp,
  rc_throttle_norm, rc_yaw_norm
"""

import rospy
import os
import csv
import time
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud, NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, RCIn
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

EARTH_RADIUS = 6371000.0  # m


class HitlPretrainPCNode:
    def __init__(self):
        rospy.init_node("hitl_pretrain_pointcloud_node")

        # ---------- PARÂMETROS ----------
        self.pose_topic = rospy.get_param("~pose_topic",
                                          "/mavros/local_position/pose")
        self.vel_topic  = rospy.get_param("~vel_topic",
                                          "/mavros/local_position/velocity_local")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.global_topic = rospy.get_param("~global_topic",
                                            "/mavros/global_position/global")
        self.wp_topic = rospy.get_param("~wp_topic",
                                        "/mavros/setpoint_raw/target_global")
        self.rc_in_topic = rospy.get_param("~rc_in_topic",
                                           "/mavros/rc/in")

        # índices dos canais no RCIn.channels (0-based)
        self.throttle_idx = rospy.get_param("~throttle_idx", 2)  # ch3
        self.yaw_idx      = rospy.get_param("~yaw_idx", 0)       # ch1

        self.stop_topic = rospy.get_param("~stop_topic",
                                          "/xasv/pretrain_stop")

        self.log_dir = rospy.get_param("~log_dir", "/tmp/xasv_hitl_pretrain")
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.max_range = rospy.get_param("~max_range", 50.0)

        os.makedirs(self.log_dir, exist_ok=True)
        tstamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir,
                                     "hitl_pretrain_pc_{}.csv".format(tstamp))

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        # CSV enxuto + normalizado
        self.writer.writerow([
            "t",
            "phase",
            "min_front", "min_left", "min_right",
            "speed", "yaw",
            "dist_to_wp", "bearing_to_wp",
            "rc_throttle_norm", "rc_yaw_norm"
        ])

        rospy.loginfo("[HITL-PC] Logging em: %s", self.csv_path)
        rospy.loginfo("[HITL-PC] LiDAR: /livox/pcl (PointCloud1)")
        rospy.loginfo("[HITL-PC] Waypoint global em: %s", self.wp_topic)
        rospy.loginfo("[HITL-PC] RC in em: %s (throttle_idx=%d, yaw_idx=%d)",
                      self.rc_in_topic, self.throttle_idx, self.yaw_idx)
        rospy.loginfo("[HITL-PC] Para parar pré-treino, pub True em: %s",
                      self.stop_topic)

        # ---------- ESTADO ----------
        self.cloud_xy = None
        self.pose = None
        self.vel = None
        self.state = None
        self.global_pos = None
        self.wp_global = None
        self.rc_in = None

        self.phase = "pretrain"
        self.stopped = False

        # ---------- SUBSCRIBERS ----------
        self.cloud_sub = rospy.Subscriber("/livox/pcl",
                                          PointCloud,
                                          self.cloud_cb)
        rospy.Subscriber(self.pose_topic,
                         PoseStamped, self.pose_cb)
        rospy.Subscriber(self.vel_topic,
                         TwistStamped, self.vel_cb)
        rospy.Subscriber(self.state_topic,
                         State, self.state_cb)
        rospy.Subscriber(self.global_topic,
                         NavSatFix, self.global_cb)
        rospy.Subscriber(self.wp_topic,
                         GlobalPositionTarget, self.wp_cb)
        rospy.Subscriber(self.rc_in_topic,
                         RCIn, self.rc_in_cb)
        rospy.Subscriber(self.stop_topic,
                         Bool, self.stop_cb)

        # ---------- PHASE PUB ----------
        self.phase_pub = rospy.Publisher("~phase", String,
                                         queue_size=1, latch=True)
        self.rate = rospy.Rate(self.rate_hz)
        self.publish_phase()

    # ====== callbacks ======
    def cloud_cb(self, msg):
        pts = [(pt.x, pt.y) for pt in msg.points]
        self.cloud_xy = np.array(pts, dtype=np.float32) if pts else None

    def pose_cb(self, msg):
        self.pose = msg

    def vel_cb(self, msg):
        self.vel = msg

    def state_cb(self, msg):
        self.state = msg

    def global_cb(self, msg):
        self.global_pos = msg

    def wp_cb(self, msg):
        self.wp_global = msg

    def rc_in_cb(self, msg):
        self.rc_in = msg

    def stop_cb(self, msg):
        if msg.data and not self.stopped:
            rospy.logwarn("[HITL-PC] Stop recebido em %s! "
                          "Encerrando pré-treino e mudando phase para 'drl'.",
                          self.stop_topic)
            self.stopped = True
            self.phase = "drl"
            self.publish_phase()
            # NÃO fecha o arquivo aqui, para evitar corrida com o loop

    # ====== helpers ======
    def publish_phase(self):
        s = String()
        s.data = self.phase
        self.phase_pub.publish(s)

    def lidar_sectors_from_cloud(self):
        if self.cloud_xy is None or self.cloud_xy.size == 0:
            r = float(self.max_range)
            return r, r, r

        x = self.cloud_xy[:, 0]
        y = self.cloud_xy[:, 1]
        r = np.sqrt(x * x + y * y)
        ang = np.arctan2(y, x)

        max_r = float(self.max_range)

        def min_sector(a_min, a_max):
            m = (ang >= a_min) & (ang <= a_max)
            if not np.any(m):
                return max_r
            vals = r[m]
            return float(np.min(vals)) if vals.size > 0 else max_r

        min_front = min_sector(-math.radians(30.0),  math.radians(30.0))
        min_left  = min_sector( math.radians(30.0),  math.radians(90.0))
        min_right = min_sector(-math.radians(90.0), -math.radians(30.0))
        return min_front, min_left, min_right

    def state_features(self):
        if self.pose is None:
            return None, None

        q = self.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        if self.vel is None:
            speed = 0.0
        else:
            vx = self.vel.twist.linear.x
            vy = self.vel.twist.linear.y
            speed = math.sqrt(vx * vx + vy * vy)

        return speed, yaw

    def global_dist_bearing(self):
        """
        Retorna (dist_to_wp, bearing_to_wp)
        dist em metros, bearing em rad (0=Norte, +p/Leste)
        """
        if self.global_pos is None or self.wp_global is None:
            return 0.0, 0.0

        veh_lat = math.radians(self.global_pos.latitude)
        veh_lon = math.radians(self.global_pos.longitude)
        wp_lat  = math.radians(self.wp_global.latitude)
        wp_lon  = math.radians(self.wp_global.longitude)

        dlat = wp_lat - veh_lat
        dlon = wp_lon - veh_lon

        dy = EARTH_RADIUS * dlat
        dx = EARTH_RADIUS * dlon * math.cos(veh_lat)

        dist = math.sqrt(dx * dx + dy * dy)
        bearing = math.atan2(dx, dy)
        return dist, bearing

    def rc_features(self):
        """
        rc_throttle_norm, rc_yaw_norm em [-1, 1]
        a partir de RCIn.channels (1000-2000, centro ~1500).

        Se ainda não tiver RC, retorna (0, 0) = neutro.
        """
        if self.rc_in is None or not self.rc_in.channels:
            return 0.0, 0.0

        ch = self.rc_in.channels
        if len(ch) <= max(self.throttle_idx, self.yaw_idx):
            return 0.0, 0.0

        rc_throttle_raw = float(ch[self.throttle_idx])
        rc_yaw_raw = float(ch[self.yaw_idx])

        def norm(v):
            # 1000 -> -1, 1500 -> 0, 2000 -> +1 (com saturação)
            return max(-1.0, min(1.0, (v - 1500.0) / 500.0))

        rc_throttle_norm = norm(rc_throttle_raw)
        rc_yaw_norm = norm(rc_yaw_raw)

        rospy.logwarn_throttle(
            5.0,
            "[HITL-PC] RC ch=%s -> thr(ch%d)=%.0f (%.2f), yaw(ch%d)=%.0f (%.2f)",
            list(ch),
            self.throttle_idx, rc_throttle_raw, rc_throttle_norm,
            self.yaw_idx,      rc_yaw_raw,      rc_yaw_norm
        )

        return rc_throttle_norm, rc_yaw_norm

    # ====== loop principal ======
    def spin(self):
        rospy.loginfo("[HITL-PC] Node iniciado. Phase: %s", self.phase)
        r = self.rate
        while not rospy.is_shutdown():
            # se já pediram pra parar ou mudou a fase, sai do loop
            if self.stopped or self.phase != "pretrain":
                rospy.loginfo("[HITL-PC] Pré-treino encerrado, saindo do loop.")
                break

            if self.state is None:
                rospy.logwarn_throttle(5.0,
                    "[HITL-PC] Aguardando state em %s", self.state_topic)
                r.sleep()
                continue

            if self.pose is None:
                rospy.logwarn_throttle(5.0,
                    "[HITL-PC] Aguardando pose em %s", self.pose_topic)
                r.sleep()
                continue

            if self.global_pos is None or self.wp_global is None:
                rospy.logwarn_throttle(5.0,
                    "[HITL-PC] Aguardando global/wp em %s / %s",
                    self.global_topic, self.wp_topic)

            if self.cloud_xy is None:
                rospy.logwarn_throttle(
                    5.0,
                    "[HITL-PC] Ainda sem nuvem em /livox/pcl "
                    "(conexões=%d)",
                    self.cloud_sub.get_num_connections()
                )

            t = rospy.get_time()

            min_front, min_left, min_right = self.lidar_sectors_from_cloud()
            speed, yaw = self.state_features()
            dist_to_wp, bearing_to_wp = self.global_dist_bearing()
            rc_throttle_norm, rc_yaw_norm = self.rc_features()

            if speed is None:
                r.sleep()
                continue

            self.writer.writerow([
                f"{t:.3f}",
                self.phase,
                f"{min_front:.4f}",
                f"{min_left:.4f}",
                f"{min_right:.4f}",
                f"{speed:.4f}",
                f"{yaw:.4f}",
                f"{dist_to_wp:.3f}",
                f"{bearing_to_wp:.4f}",
                f"{rc_throttle_norm:.3f}",
                f"{rc_yaw_norm:.3f}",
            ])
            self.csv_file.flush()

            r.sleep()

        if not self.csv_file.closed:
            self.csv_file.close()
        rospy.loginfo("[HITL-PC] Encerrado.")


if __name__ == "__main__":
    try:
        node = HitlPretrainPCNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

