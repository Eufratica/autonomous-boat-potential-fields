#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Controle de Desvio de Obstaculos por Campos Potenciais Artificiais (APF) para Barco ASV
Baseado no algoritmo de campos potenciais de iq_gnc (obs_avoid.py / avoidance.cpp)
Adaptado para cinematica de superficie aquatica nao-holonomica (Surge / Yaw).
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped, Wrench, Point, Vector3
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion

class BoatPotentialFields:
    def __init__(self):
        rospy.init_node("boat_potential_fields", anonymous=False)
        rospy.loginfo("[APF] Inicializando no de Campos Potenciais para Barco ASV...")

        # --- Parametros Configuraveis ---
        self.k_att = rospy.get_param("~k_att", 1.5)           # Ganho atrativo
        self.k_rep = rospy.get_param("~k_rep", 1.0)           # Ganho repulsivo
        self.d0 = rospy.get_param("~d0", 5.0)                 # Raio de influencia de obstaculo (m)
        self.d_min = rospy.get_param("~d_min", 0.35)          # Distancia minima valida (m)
        self.d_critical = rospy.get_param("~d_critical", 1.2) # Distancia critica de emergencia (m)
        
        self.k_yaw = rospy.get_param("~k_yaw", 2.0)           # Ganho proporcional de guinada
        self.max_surge = rospy.get_param("~max_surge", 0.8)   # Empuxo maximo de avanco [0..1]
        self.max_torque = rospy.get_param("~max_torque", 1.0) # Torque maximo de giro [-1..1]
        self.goal_dist_thresh = rospy.get_param("~goal_dist_thresh", 2.5) # Raio para aceitar waypoint (m)

        self.robot_name = rospy.get_param("~robot_name", "migbot1")

        # Lista de Waypoints na arena aquatica fechada (X, Y)
        # Posicao inicial aproximada: X=5.0, Y=10.0
        self.waypoints = [
            (20.0, 0.0),
            (35.0, 0.0),
            (55.0, 0.0),
            (40.0, 12.0),
            (20.0, 12.0),
            (5.0, 0.0)
        ]
        self.current_wp_idx = 0

        # Estado do Barco
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_pose = False

        # Vetores de Forca
        self.f_att = np.zeros(2)
        self.f_rep = np.zeros(2)
        self.f_total = np.zeros(2)
        self.min_front_dist = 999.0

        # Publicadores
        self.pub_wrench = rospy.Publisher("Wrench", Wrench, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_mavros = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        self.pub_markers = rospy.Publisher("/potential_fields/markers", MarkerArray, queue_size=1)

        # Assinantes
        rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_states_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        self.rate = rospy.Rate(20) # 20 Hz
        rospy.loginfo("[APF] Pronto! Navegando para o primeiro waypoint: %s", str(self.waypoints[0]))

    def gazebo_states_cb(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            pose = msg.pose[idx]
            self.x = pose.position.x
            self.y = pose.position.y
            q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            _, _, self.yaw = euler_from_quaternion(q)
            self.has_pose = True

    def odom_cb(self, msg):
        if not self.has_pose:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            _, _, self.yaw = euler_from_quaternion(q)
            self.has_pose = True

    def laser_cb(self, msg):
        """
        Calcula a Forca Repulsiva (F_rep) baseada na equacao do iq_gnc:
        U = -0.5 * k * (1/d - 1/d0)^2
        """
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filtrar distancias validas no campo de visao frontal e lateral (+/- 120 graus)
        front_lateral = np.abs(angles) <= math.radians(120.0)
        valid_mask = (ranges > self.d_min) & (ranges < self.d0) & front_lateral & ~np.isinf(ranges) & ~np.isnan(ranges)
        
        rep_x_body = 0.0
        rep_y_body = 0.0
        min_front = 999.0

        if np.any(valid_mask):
            v_ranges = ranges[valid_mask]
            v_angles = angles[valid_mask]

            # Deteccao de obstaculo no cone frontal (+/- 30 graus)
            front_mask = np.abs(v_angles) < math.radians(35.0)
            if np.any(front_mask):
                min_front = np.min(v_ranges[front_mask])

            # Formula de Potencial Repulsivo (mesma de obs_avoid.py)
            # U = -0.5 * k * (1/d - 1/d0)^2
            u = -0.5 * self.k_rep * np.power((1.0 / v_ranges - 1.0 / self.d0), 2.0)
            
            # Decomposicao nos eixos do corpo do barco
            # Nota: O sinal negativo em U empurra o barco na direcao OPOSTA ao obstaculo
            rep_x_body = np.sum(np.cos(v_angles) * u)
            rep_y_body = np.sum(np.sin(v_angles) * u)

        self.min_front_dist = min_front

        # Rotacionar do frame local (body) para o frame global (world)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        rep_x_global = rep_x_body * cos_yaw - rep_y_body * sin_yaw
        rep_y_global = rep_x_body * sin_yaw + rep_y_body * cos_yaw

        # Limitar magnitude repulsiva
        rep_mag = math.hypot(rep_x_global, rep_y_global)
        max_rep = 4.0
        if rep_mag > max_rep:
            rep_x_global = max_rep * (rep_x_global / rep_mag)
            rep_y_global = max_rep * (rep_y_global / rep_mag)

        self.f_rep = np.array([rep_x_global, rep_y_global])

    def compute_attractive_force(self):
        """Calcula a forca atrativa em direcao ao waypoint atual."""
        if not self.has_pose:
            return np.zeros(2)

        goal_x, goal_y = self.waypoints[self.current_wp_idx]
        dx = goal_x - self.x
        dy = goal_y - self.y
        dist = math.hypot(dx, dy)

        if dist < self.goal_dist_thresh:
            rospy.loginfo("[APF] Waypoint %d atingido! (%0.1f, %0.1f)", self.current_wp_idx, goal_x, goal_y)
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            goal_x, goal_y = self.waypoints[self.current_wp_idx]
            dx = goal_x - self.x
            dy = goal_y - self.y
            dist = math.hypot(dx, dy)

        # Forca atrativa proporcional com saturacao suave
        f_mag = min(self.k_att * dist, 3.0)
        if dist > 0.01:
            att_x = f_mag * (dx / dist)
            att_y = f_mag * (dy / dist)
        else:
            att_x, att_y = 0.0, 0.0

        return np.array([att_x, att_y])

    def control_loop(self):
        while not rospy.is_shutdown():
            if not self.has_pose:
                rospy.logwarn_throttle(2.0, "[APF] Aguardando telemetria de pose do barco...")
                self.rate.sleep()
                continue

            # 1. Calcular Forcas
            self.f_att = self.compute_attractive_force()
            self.f_total = self.f_att + self.f_rep

            # 2. Desvio de Minimo Local (escape caso resultante anule perto de obstaculo)
            total_mag = math.hypot(self.f_total[0], self.f_total[1])
            if total_mag < 0.2 and np.linalg.norm(self.f_rep) > 0.5:
                # Adiciona componente ortogonal para quebrar simetria
                rospy.logwarn_throttle(1.0, "[APF] Potencial minimo local detectado! Aplicando forca de escape.")
                escape_vec = np.array([-self.f_rep[1], self.f_rep[0]])
                self.f_total += 1.0 * (escape_vec / np.linalg.norm(escape_vec))

            # 3. Orientacao Desejada da Proa
            desired_yaw = math.atan2(self.f_total[1], self.f_total[0])
            yaw_error = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))

            # 4. Controle Dinamico do Barco
            # - Torque de Guinada (leme / diferenca de empuxo)
            torque_z = np.clip(self.k_yaw * yaw_error, -self.max_torque, self.max_torque)

            # - Forca Longitudinal (Surge / aceleracao)
            # Reduz velocidade quando a curva for fechada para estabilidade
            angle_factor = max(0.0, math.cos(yaw_error))
            surge_fx = self.max_surge * angle_factor

            # Manobra de Emergencia / Recuo se o obstaculo estiver critico na proa
            if self.min_front_dist < self.d_critical:
                rospy.logwarn_throttle(0.5, "[APF] CUIDADO! Obstaculo critico a %.2fm. Reduzindo e manobrando!", self.min_front_dist)
                surge_fx = 0.1 # minima velocidade para virar
                if self.min_front_dist < 0.7:
                    surge_fx = -0.3 # marcha a re defensiva
                torque_z = math.copysign(self.max_torque, yaw_error if abs(yaw_error) > 0.1 else 1.0)

            # 5. Publicacao dos Comandos
            # A) Wrench para migbot_allocation (propulsores fisicos no Gazebo)
            wrench_msg = Wrench()
            wrench_msg.force.x = surge_fx
            wrench_msg.torque.z = torque_z
            self.pub_wrench.publish(wrench_msg)

            # B) Twist convencional (/cmd_vel)
            twist_msg = Twist()
            twist_msg.linear.x = surge_fx * 2.0  # escala m/s
            twist_msg.angular.z = torque_z * 1.0 # rad/s
            self.pub_cmd_vel.publish(twist_msg)

            # C) TwistStamped para MAVROS (ArduPilot Rover GUIDED mode)
            ts_msg = TwistStamped()
            ts_msg.header.stamp = rospy.Time.now()
            ts_msg.header.frame_id = "base_link"
            ts_msg.twist = twist_msg
            self.pub_mavros.publish(ts_msg)

            # 6. Publicar RViz Markers para inspecao visual
            self.publish_markers()

            self.rate.sleep()

    def publish_markers(self):
        marker_arr = MarkerArray()
        
        # Marcador do Waypoint Atual
        wp_x, wp_y = self.waypoints[self.current_wp_idx]
        wp_m = Marker()
        wp_m.header.frame_id = "world"
        wp_m.header.stamp = rospy.Time.now()
        wp_m.ns = "waypoint"
        wp_m.id = 0
        wp_m.type = Marker.SPHERE
        wp_m.action = Marker.ADD
        wp_m.pose.position.x = wp_x
        wp_m.pose.position.y = wp_y
        wp_m.pose.position.z = 0.5
        wp_m.scale.x = 1.5
        wp_m.scale.y = 1.5
        wp_m.scale.z = 1.5
        wp_m.color.r = 1.0
        wp_m.color.g = 0.8
        wp_m.color.b = 0.0
        wp_m.color.a = 0.8
        marker_arr.markers.append(wp_m)

        # Vetor Atrativo (Verde)
        m_att = Marker()
        m_att.header.frame_id = "world"
        m_att.header.stamp = rospy.Time.now()
        m_att.ns = "forces"
        m_att.id = 1
        m_att.type = Marker.ARROW
        m_att.action = Marker.ADD
        p1 = Point(self.x, self.y, 0.5)
        p2 = Point(self.x + self.f_att[0], self.y + self.f_att[1], 0.5)
        m_att.points = [p1, p2]
        m_att.scale.x = 0.15 # diametro da haste
        m_att.scale.y = 0.3  # diametro da cabeca
        m_att.color.g = 1.0
        m_att.color.a = 0.9
        marker_arr.markers.append(m_att)

        # Vetor Repulsivo (Vermelho)
        m_rep = Marker()
        m_rep.header.frame_id = "world"
        m_rep.header.stamp = rospy.Time.now()
        m_rep.ns = "forces"
        m_rep.id = 2
        m_rep.type = Marker.ARROW
        m_rep.action = Marker.ADD
        p3 = Point(self.x + self.f_rep[0], self.y + self.f_rep[1], 0.5)
        m_rep.points = [p1, p3]
        m_rep.scale.x = 0.15
        m_rep.scale.y = 0.3
        m_rep.color.r = 1.0
        m_rep.color.a = 0.9
        marker_arr.markers.append(m_rep)

        # Vetor Resultante Total (Azul)
        m_tot = Marker()
        m_tot.header.frame_id = "world"
        m_tot.header.stamp = rospy.Time.now()
        m_tot.ns = "forces"
        m_tot.id = 3
        m_tot.type = Marker.ARROW
        m_tot.action = Marker.ADD
        p4 = Point(self.x + self.f_total[0], self.y + self.f_total[1], 0.5)
        m_tot.points = [p1, p4]
        m_tot.scale.x = 0.2
        m_tot.scale.y = 0.4
        m_tot.color.b = 1.0
        m_tot.color.a = 1.0
        marker_arr.markers.append(m_tot)

        self.pub_markers.publish(marker_arr)

if __name__ == "__main__":
    try:
        node = BoatPotentialFields()
        node.control_loop()
    except rospy.ROSInterruptException:
        pass
