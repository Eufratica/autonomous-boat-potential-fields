#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xasv_hitl_policy_node.py

Node de EXECUÇÃO da política HITL supervisionada.

- Carrega pretrained_rc_policy.pt (treinado em train_supervised.py).
- Lê LiDAR, pose, velocidade, posição global e waypoint, monta o mesmo vetor
  de estado do pré-treino:

    state = [
        min_front, min_left, min_right,
        speed, yaw,
        dist_to_wp, bearing_to_wp
    ]

- Quando há obstáculo à frente, aplica RC override (throttle / yaw)
  usando a MLP.
- Quando está livre de obstáculos, limpa o override (ArduPilot controla).

- IMPORTANTE:
    * seta SYSID_MYGCS via MAVROS para habilitar RC_OVERRIDE (por padrão).
    * em linha com o treino atual, priorizamos yaw sobre throttle.
"""

import rospy
import math
import os
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import PointCloud, NavSatFix
from mavros_msgs.msg import (
    State,
    GlobalPositionTarget,
    OverrideRCIn,
    ParamValue,
)
from mavros_msgs.srv import ParamSet, ParamPull
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

import torch
import torch.nn as nn

EARTH_RADIUS = 6371000.0  # m


# ==== MLP igual à usada no treino ====

class PolicyMLP(nn.Module):
    def __init__(self, in_dim, hidden=64, out_dim=2):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, out_dim)
        )

    def forward(self, x):
        return self.net(x)


class XasvHitlPolicyNode:
    def __init__(self):
        rospy.init_node("xasv_hitl_policy_node")

        # --- tópicos de estado ---
        self.pose_topic   = rospy.get_param("~pose_topic",   "/mavros/local_position/pose")
        self.vel_topic    = rospy.get_param("~vel_topic",    "/mavros/local_position/velocity_local")
        self.state_topic  = rospy.get_param("~state_topic",  "/mavros/state")
        self.global_topic = rospy.get_param("~global_topic", "/mavros/global_position/global")
        self.wp_topic     = rospy.get_param("~wp_topic",     "/mavros/setpoint_raw/target_global")
        self.collision_topic = rospy.get_param("~collision_topic", "/xasv/collision_flag")

        # --- RC override ---
        self.rc_override_topic = rospy.get_param("~rc_override_topic", "/mavros/rc/override")
        self.throttle_idx = rospy.get_param("~throttle_idx", 2)  # RC3
        self.yaw_idx      = rospy.get_param("~yaw_idx", 0)       # RC1
        self.control_enabled = rospy.get_param("~control_enabled", True)

        # --- LiDAR / gating ---
        self.safe_dist = rospy.get_param("~safe_dist", 5.0)             # distância "perigosa"
        self.min_valid_range = rospy.get_param("~min_valid_range", 1.0) # ignora hits muito perto
        self.front_points_min = rospy.get_param("~front_points_min", 5) # nº mínimo de pontos na frente
        self.yaw_deadband = rospy.get_param("~yaw_deadband", 0.1)       # zona morta no yaw

        # --- pós-processamento da ação da rede ---
        self.thr_scale = rospy.get_param("~thr_scale", 1.0)         # escala saída de throttle
        self.yaw_scale = rospy.get_param("~yaw_scale", 1.0)         # escala saída de yaw
        self.allow_reverse = rospy.get_param("~allow_reverse", False)  # se False, thr>=0

        self.rate_hz = rospy.get_param("~rate", 10.0)

        # --- SYSID_MYGCS via MAVROS ---
        #self.set_sysid_mygcs_flag = rospy.get_param("~set_sysid_mygcs", True)
        #self.sysid_mygcs_value = rospy.get_param("~sysid_mygcs_value", 1)  # default 1

        # --- carrega modelo pré-treinado ---
        self.model_path = rospy.get_param("~model_path", "trained_rc_policy.pt")
        self.device = torch.device("cpu")

        if not os.path.isfile(self.model_path):
            rospy.logerr("[HITL-POL] Modelo %s não encontrado!", self.model_path)
            raise SystemExit(1)

        ckpt = torch.load(self.model_path, map_location=self.device)

        # in_dim = 7 (features fixas)
        self.in_dim = 7
        self.policy = PolicyMLP(in_dim=self.in_dim, hidden=64, out_dim=2).to(self.device)
        self.policy.load_state_dict(ckpt["state_dict"])
        self.policy.eval()

        feat_cols = ckpt.get("feature_cols", [])
        rospy.loginfo("[HITL-POL] Modelo carregado de %s", self.model_path)
        rospy.loginfo("[HITL-POL] feature_cols no checkpoint: %s", feat_cols)

        # --- estado interno ---
        self.cloud_xy = None
        self.pose = None
        self.vel = None
        self.state_msg = None
        self.global_pos = None
        self.wp_global = None
        self.collision = False

        # --- subs / pubs ---
        self.cloud_sub = rospy.Subscriber("/livox/pcl", PointCloud, self.cloud_cb)
        rospy.Subscriber(self.pose_topic,   PoseStamped,          self.pose_cb)
        rospy.Subscriber(self.vel_topic,    TwistStamped,         self.vel_cb)
        rospy.Subscriber(self.state_topic,  State,                self.state_cb)
        rospy.Subscriber(self.global_topic, NavSatFix,            self.global_cb)
        rospy.Subscriber(self.wp_topic,     GlobalPositionTarget, self.wp_cb)
        rospy.Subscriber(self.collision_topic, Bool,              self.collision_cb)

        self.rc_override_pub = rospy.Publisher(self.rc_override_topic,
                                               OverrideRCIn,
                                               queue_size=1)

        self.rate = rospy.Rate(self.rate_hz)
        rospy.on_shutdown(self.on_shutdown)

        # tenta setar SYSID_MYGCS
        #if self.set_sysid_mygcs_flag:
        #    self.set_sysid_mygcs()

        rospy.loginfo(
            "[HITL-POL] Node iniciado. control_enabled=%s, thr_scale=%.2f, yaw_scale=%.2f, allow_reverse=%s",
            str(self.control_enabled), self.thr_scale, self.yaw_scale, str(self.allow_reverse)
        )

    # ===== SYSID_MYGCS =====
    '''
    def set_sysid_mygcs(self):
        """
        Tenta setar SYSID_MYGCS via MAVROS:
          1) /mavros/param/pull
          2) /mavros/param/set  SYSID_MYGCS = self.sysid_mygcs_value
        """
        pull_srv = "/mavros/param/pull"
        set_srv  = "/mavros/param/set"

        # 1) PULL
        try:
            rospy.loginfo("[HITL-POL] Aguardando serviço %s...", pull_srv)
            rospy.wait_for_service(pull_srv, timeout=30.0)
        except rospy.ROSException:
            rospy.logwarn("[HITL-POL] Serviço %s não disponível, não foi possível puxar parâmetros.",
                          pull_srv)
            return

        try:
            ParamPullSrv = rospy.ServiceProxy(pull_srv, ParamPull)
            resp_pull = ParamPullSrv(force_pull=True)
            if resp_pull.success:
                rospy.loginfo("[HITL-POL] Param pull OK (%d parâmetros recebidos).",
                              resp_pull.param_received)
            else:
                rospy.logwarn("[HITL-POL] Param pull falhou (success=false).")
        except rospy.ServiceException as e:
            rospy.logwarn("[HITL-POL] Erro chamando %s: %s", pull_srv, str(e))
            return

        # 2) SET SYSID_MYGCS
        try:
            rospy.loginfo("[HITL-POL] Aguardando serviço %s para setar SYSID_MYGCS...", set_srv)
            rospy.wait_for_service(set_srv, timeout=30.0)
        except rospy.ROSException:
            rospy.logwarn("[HITL-POL] Serviço %s não disponível, não foi possível setar SYSID_MYGCS.",
                          set_srv)
            return

        try:
            ParamSetSrv = rospy.ServiceProxy(set_srv, ParamSet)
            val = ParamValue(integer=int(self.sysid_mygcs_value))
            resp_set = ParamSetSrv(param_id="SYSID_MYGCS", value=val)

            if resp_set.success:
                rospy.loginfo("[HITL-POL] SYSID_MYGCS setado para %d via %s",
                              resp_set.value.integer, set_srv)
            else:
                rospy.logwarn("[HITL-POL] Falha ao setar SYSID_MYGCS via %s (success=false)", set_srv)

        except rospy.ServiceException as e:
            rospy.logwarn("[HITL-POL] Erro chamando %s: %s", set_srv, str(e))
    '''     

    # ===== callbacks =====
    def cloud_cb(self, msg):
        pts = [(pt.x, pt.y) for pt in msg.points]
        self.cloud_xy = np.array(pts, dtype=np.float32) if pts else None

    def pose_cb(self, msg):
        self.pose = msg

    def vel_cb(self, msg):
        self.vel = msg

    def state_cb(self, msg):
        self.state_msg = msg

    def global_cb(self, msg):
        self.global_pos = msg

    def wp_cb(self, msg):
        self.wp_global = msg

    def collision_cb(self, msg):
        self.collision = bool(msg.data)

    # ===== helpers de estado / LiDAR =====
    def lidar_sectors(self):
        """
        Retorna: min_front, min_left, min_right, n_front_close.

        - min_* em metros.
        - n_front_close = nº de pontos no setor frontal com
          min_valid_range <= r < safe_dist.
        """
        if self.cloud_xy is None or self.cloud_xy.size == 0:
            r = self.safe_dist * 2.0
            return r, r, r, 0

        x = self.cloud_xy[:, 0]
        y = self.cloud_xy[:, 1]
        r = np.sqrt(x * x + y * y)
        ang = np.arctan2(y, x)

        # ignora retornos muito próximos (mesh do próprio sensor, etc.)
        valid_mask = r >= self.min_valid_range
        if not np.any(valid_mask):
            r_default = self.safe_dist * 2.0
            return r_default, r_default, r_default, 0

        x = x[valid_mask]
        y = y[valid_mask]
        r = r[valid_mask]
        ang = ang[valid_mask]

        max_r = 50.0

        f_mask = (ang >= -math.radians(30)) & (ang <= math.radians(30))
        l_mask = (ang >=  math.radians(30)) & (ang <= math.radians(90))
        r_mask = (ang >= -math.radians(90)) & (ang <= -math.radians(30))

        def min_with_mask(mask, default):
            if not np.any(mask):
                return default
            vals = r[mask]
            return float(vals.min()) if vals.size > 0 else default

        min_front = min_with_mask(f_mask, max_r)
        min_left  = min_with_mask(l_mask, max_r)
        min_right = min_with_mask(r_mask, max_r)

        f_close_mask = f_mask & (r < self.safe_dist)
        n_front_close = int(np.sum(f_close_mask))

        return min_front, min_left, min_right, n_front_close

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
        dist em metros, bearing em rad (0 = Norte, + para Leste).
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

    def build_state(self):
        """
        Retorna:
          state_vec (np.array 7D),
          dist_to_wp, min_front, n_front_close
        ou (None, ...) se ainda falta info essencial.
        """
        if self.state_msg is None or self.pose is None:
            rospy.logwarn_throttle(
                5.0,
                "[HITL-POL] Aguardando pose/state (pose=%s, state=%s)",
                bool(self.pose), bool(self.state_msg)
            )
            return None, None, None, None

        min_front, min_left, min_right, n_front_close = self.lidar_sectors()
        speed, yaw = self.state_features()
        dist_to_wp, bearing_to_wp = self.global_dist_bearing()

        state_vec = np.array([
            min_front, min_left, min_right,
            speed, yaw,
            dist_to_wp, bearing_to_wp
        ], dtype=np.float32)

        return state_vec, dist_to_wp, min_front, n_front_close

    # ===== RC override =====
    def publish_rc_override(self, thr_norm, yaw_norm):
        msg = OverrideRCIn()
        msg.channels = [0] * 18  # 0 = no override

        def to_pwm(a):
            v = 1500.0 + 500.0 * a
            v = max(1000.0, min(2000.0, v))
            return int(v)

        thr_pwm = to_pwm(thr_norm)
        yaw_pwm = to_pwm(yaw_norm)

        if 0 <= self.throttle_idx < len(msg.channels):
            msg.channels[self.throttle_idx] = thr_pwm
        if 0 <= self.yaw_idx < len(msg.channels):
            msg.channels[self.yaw_idx] = yaw_pwm

        rospy.logwarn_throttle(
            2.0,
            "[HITL-POL] RC override -> thr(ch%d)=%.2f (%d), yaw(ch%d)=%.2f (%d)",
            self.throttle_idx, thr_norm, thr_pwm,
            self.yaw_idx,      yaw_norm, yaw_pwm
        )

        self.rc_override_pub.publish(msg)

    def publish_rc_clear(self):
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        self.rc_override_pub.publish(msg)
        rospy.logwarn_throttle(5.0, "[HITL-POL] RC override CLEAR (livre).")

    def on_shutdown(self):
        try:
            self.publish_rc_clear()
        except Exception:
            pass
        rospy.loginfo("[HITL-POL] Encerrado.")

    # ===== loop principal =====
    def spin(self):
        while not rospy.is_shutdown():
            state, dist, min_front, n_front_close = self.build_state()
            if state is None:
                self.rate.sleep()
                continue

            # só atua em AUTO/GUIDED (pra não atrapalhar MANUAL/RTL etc.)
            mode = (self.state_msg.mode or "").upper() if self.state_msg else ""
            if mode not in ["AUTO", "GUIDED"]:
                self.publish_rc_clear()
                self.rate.sleep()
                continue

            # gating por LiDAR: precisa de obstáculo REAL na frente
            need_avoid = (min_front < self.safe_dist and
                          n_front_close is not None and
                          n_front_close >= self.front_points_min)

            rospy.logwarn_throttle(
                2.0,
                "[HITL-POL] gating LiDAR: min_front=%.2f, n_front_close=%d, "
                "safe_dist=%.2f, front_points_min=%d, need_avoid=%s",
                float(min_front), int(n_front_close),
                float(self.safe_dist), int(self.front_points_min),
                str(need_avoid)
            )

            if (not need_avoid) or self.collision or (not self.control_enabled):
                # caminho livre, sem controle, ou já colidiu → deixa FCU tocar
                self.publish_rc_clear()
                self.rate.sleep()
                continue

            # monta tensor de entrada e passa pela MLP
            inp = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
            with torch.no_grad():
                out = self.policy(inp)[0].cpu().numpy()

            thr_norm = float(np.clip(out[0], -1.0, 1.0))
            yaw_norm = float(np.clip(out[1], -1.0, 1.0))

            # aplica escalas configuráveis
            thr_norm *= self.thr_scale
            yaw_norm *= self.yaw_scale
            thr_norm = float(np.clip(thr_norm, -1.0, 1.0))
            yaw_norm = float(np.clip(yaw_norm, -1.0, 1.0))

            # opcional: não permitir throttle reverso
            if (not self.allow_reverse) and thr_norm < 0.0:
                thr_norm = 0.0

            # deadband no yaw pra evitar oscilação pequena
            if abs(yaw_norm) < self.yaw_deadband:
                yaw_norm = 0.0

            # opcional: reduzir throttle quanto mais perto do obstáculo
            if min_front < self.safe_dist:
                # mapeia [1 m, safe_dist] -> [0, 1], saturando
                if self.safe_dist > 1.0:
                    scale = max(0.0, min(1.0, (min_front - 1.0) / (self.safe_dist - 1.0)))
                else:
                    scale = 1.0
                thr_norm = thr_norm * scale

            self.publish_rc_override(thr_norm, yaw_norm)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = XasvHitlPolicyNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

