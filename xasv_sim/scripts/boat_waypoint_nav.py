#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script de Navegacao por Waypoints Interativa com Campos Potenciais Artificiais (APF)
Combina o clique '2D Nav Goal' do RViz (Forca Atrativa) com o desvio de boias via LiDAR (Forca Repulsiva).
Baseado no algoritmo APF do iq_gnc adaptado para cinematica de barco ASV.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion

class BoatWaypointAPFNav:
    def __init__(self):
        rospy.init_node("boat_waypoint_nav", anonymous=False)
        rospy.loginfo("======================================================")
        rospy.loginfo("[Waypoint APF Nav] Inicializando Navegacao com Desvio por Campos Potenciais...")

        # --- Parametros de Navegacao ---
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.85)  # m/s
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.2) # rad/s
        self.k_p_yaw = rospy.get_param("~k_p_yaw", 1.8)                    # Ganho proporcional de guinada
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.0)       # Raio de chegada (m)
        self.slowdown_dist = rospy.get_param("~slowdown_dist", 3.0)         # Distancia de desaceleracao (m)
        self.robot_name = rospy.get_param("~robot_name", "migbot1")

        # --- Parametros de Campos Potenciais (APF / iq_gnc) ---
        self.k_att = rospy.get_param("~k_att", 1.5)            # Ganho atrativo
        self.k_rep = rospy.get_param("~k_rep", 0.9)            # Ganho repulsivo (iq_gnc)
        self.d0 = rospy.get_param("~d0", 4.5)                  # Raio de influencia dos obstaculos (m)
        self.d_min = rospy.get_param("~d_min", 0.35)           # Distancia minima valida do laser (m)
        self.d_critical = rospy.get_param("~d_critical", 1.3)  # Distancia critica frontal de alerta (m)
        self.max_rep = rospy.get_param("~max_rep", 3.5)        # Limite maximo da forca repulsiva

        # --- Estado do Barco ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.has_pose = False

        # --- Estado do Objetivo ---
        self.goal_x = None
        self.goal_y = None
        self.has_active_goal = False

        # --- Vetores de Forca (APF) ---
        self.f_att = np.zeros(2)
        self.f_rep = np.zeros(2)
        self.f_total = np.zeros(2)
        self.min_front_dist = 999.0

        # --- Publicadores ---
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_target_marker = rospy.Publisher("/waypoint_nav/target_marker", Marker, queue_size=1)
        self.pub_line_marker = rospy.Publisher("/waypoint_nav/target_line", Marker, queue_size=1)
        self.pub_force_markers = rospy.Publisher("/waypoint_nav/force_markers", MarkerArray, queue_size=1)
        self.pub_path = rospy.Publisher("/waypoint_nav/driven_path", Path, queue_size=10)

        # Historico de caminho para o RViz
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # --- Assinantes ---
        # 1. 2D Nav Goal do RViz
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/waypoint_nav/set_goal", Point, self.point_goal_cb, queue_size=1)

        # 2. LiDAR 2D do Barco (/scan)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=1)

        # 3. Telemetria de Pose do Barco
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_cb, queue_size=1)

        self.rate = rospy.Rate(20) # 20 Hz
        rospy.loginfo("[Waypoint APF Nav] Pronto! Clique em '2D Nav Goal' no RViz para iniciar navegacao.")
        rospy.loginfo("======================================================")

    def goal_cb(self, msg):
        """Acionado ao clicar em '2D Nav Goal' no RViz."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_active_goal = True
        rospy.loginfo("[Waypoint APF Nav] >>> NOVO OBJETIVO: (X=%.2f, Y=%.2f)", self.goal_x, self.goal_y)
        self.publish_target_marker()

    def point_goal_cb(self, msg):
        """Envio de waypoint via topico Point."""
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.has_active_goal = True
        rospy.loginfo("[Waypoint APF Nav] >>> NOVO OBJETIVO via /set_goal: (X=%.2f, Y=%.2f)", self.goal_x, self.goal_y)
        self.publish_target_marker()

    def odom_cb(self, msg):
        """Atualiza a pose a partir da odometria."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, self.yaw = euler_from_quaternion(q)
        self.has_pose = True

        # Atualizar rastro do caminho
        if len(self.path_msg.poses) == 0 or math.hypot(self.x - self.path_msg.poses[-1].pose.position.x,
                                                       self.y - self.path_msg.poses[-1].pose.position.y) > 0.3:
            p = PoseStamped()
            p.header.frame_id = "odom"
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = self.x
            p.pose.position.y = self.y
            p.pose.position.z = 0.1
            self.path_msg.poses.append(p)
            self.path_msg.header.stamp = rospy.Time.now()
            self.pub_path.publish(self.path_msg)

    def gazebo_cb(self, msg):
        """Fallback para telemetria de pose."""
        if not self.has_pose and self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            pose = msg.pose[idx]
            self.x = pose.position.x
            self.y = pose.position.y
            q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            _, _, self.yaw = euler_from_quaternion(q)
            self.has_pose = True

    def laser_cb(self, msg):
        """
        Calcula a Forca Repulsiva (F_rep) baseada na equacao do iq_gnc:
        U = -0.5 * k * (1/d - 1/d0)^2
        """
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filtrar feixes validos no cone frontal e lateral (+/- 120 graus)
        front_lateral = np.abs(angles) <= math.radians(120.0)
        valid = (ranges > self.d_min) & (ranges < self.d0) & front_lateral & ~np.isinf(ranges) & ~np.isnan(ranges)

        rep_x_body = 0.0
        rep_y_body = 0.0
        min_front = 999.0

        if np.any(valid):
            v_ranges = ranges[valid]
            v_angles = angles[valid]

            # Deteccao de obstaculo no cone frontal (+/- 35 graus)
            front_cone = np.abs(v_angles) < math.radians(35.0)
            if np.any(front_cone):
                min_front = float(np.min(v_ranges[front_cone]))

            # Formula de Potencial Repulsivo do iq_gnc
            # U = -0.5 * k * ((1/d) - (1/d0))^2
            u = -0.5 * self.k_rep * np.power(((1.0 / v_ranges) - (1.0 / self.d0)), 2.0)

            # Decomposicao nos eixos do barco (U negativo empurra na direcao oposta)
            rep_x_body = float(np.sum(np.cos(v_angles) * u))
            rep_y_body = float(np.sum(np.sin(v_angles) * u))

        self.min_front_dist = min_front

        # Rotacionar do frame do barco (body) para o frame global (odom)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        rep_x_odom = rep_x_body * cos_yaw - rep_y_body * sin_yaw
        rep_y_odom = rep_x_body * sin_yaw + rep_y_body * cos_yaw

        # Saturar forca repulsiva maxima
        rep_mag = math.hypot(rep_x_odom, rep_y_odom)
        if rep_mag > self.max_rep:
            rep_x_odom = self.max_rep * (rep_x_odom / rep_mag)
            rep_y_odom = self.max_rep * (rep_y_odom / rep_mag)

        self.f_rep = np.array([rep_x_odom, rep_y_odom])

    def compute_attractive_force(self):
        """Calcula a forca atrativa (F_att) em direcao ao waypoint clicado no RViz."""
        if not self.has_active_goal or self.goal_x is None:
            return np.zeros(2)

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        if dist <= 0.05:
            return np.zeros(2)

        # Forca atrativa proporcional com saturacao suave
        f_mag = min(self.k_att * dist, 3.0)
        att_x = f_mag * (dx / dist)
        att_y = f_mag * (dy / dist)
        return np.array([att_x, att_y])

    def publish_target_marker(self):
        """Desenha a coluna luminosa do waypoint no RViz."""
        if self.goal_x is None:
            return
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 2.0
        marker.color.r = 0.0
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 0.85
        self.pub_target_marker.publish(marker)

    def publish_target_line(self):
        """Linha visual guia do barco ao waypoint."""
        if not self.has_active_goal or self.goal_x is None:
            return
        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = rospy.Time.now()
        line.ns = "target_line"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.12
        line.color.r = 1.0
        line.color.g = 0.8
        line.color.b = 0.0
        line.color.a = 0.8
        line.points = [Point(self.x, self.y, 0.2), Point(self.goal_x, self.goal_y, 0.2)]
        self.pub_line_marker.publish(line)

    def publish_force_markers(self):
        """
        Publica setas 3D no RViz mostrando as forcas do APF atuando no barco:
        - Verde: Forca Atrativa (em direcao ao waypoint)
        - Vermelha: Forca Repulsiva (fugindo das boias)
        - Azul: Forca Resultante (rumo de navegacao)
        """
        arr = MarkerArray()
        origin = Point(self.x, self.y, 0.5)

        # 1. Seta Atrativa (Verde)
        att_arrow = Marker()
        att_arrow.header.frame_id = "odom"
        att_arrow.header.stamp = rospy.Time.now()
        att_arrow.ns = "forces"
        att_arrow.id = 10
        att_arrow.type = Marker.ARROW
        att_arrow.action = Marker.ADD
        att_arrow.scale.x = 0.08  # diametro do corpo
        att_arrow.scale.y = 0.18  # diametro da ponta
        att_arrow.scale.z = 0.25  # comprimento da ponta
        att_arrow.color.r = 0.1
        att_arrow.color.g = 1.0
        att_arrow.color.b = 0.1
        att_arrow.color.a = 0.9
        att_end = Point(self.x + self.f_att[0] * 0.8, self.y + self.f_att[1] * 0.8, 0.5)
        att_arrow.points = [origin, att_end]
        arr.markers.append(att_arrow)

        # 2. Seta Repulsiva (Vermelha)
        if np.linalg.norm(self.f_rep) > 0.05:
            rep_arrow = Marker()
            rep_arrow.header.frame_id = "odom"
            rep_arrow.header.stamp = rospy.Time.now()
            rep_arrow.ns = "forces"
            rep_arrow.id = 11
            rep_arrow.type = Marker.ARROW
            rep_arrow.action = Marker.ADD
            rep_arrow.scale.x = 0.08
            rep_arrow.scale.y = 0.18
            rep_arrow.scale.z = 0.25
            rep_arrow.color.r = 1.0
            rep_arrow.color.g = 0.1
            rep_arrow.color.b = 0.1
            rep_arrow.color.a = 0.95
            rep_end = Point(self.x + self.f_rep[0] * 0.8, self.y + self.f_rep[1] * 0.8, 0.5)
            rep_arrow.points = [origin, rep_end]
            arr.markers.append(rep_arrow)

        # 3. Seta Resultante (Azul)
        if np.linalg.norm(self.f_total) > 0.05:
            tot_arrow = Marker()
            tot_arrow.header.frame_id = "odom"
            tot_arrow.header.stamp = rospy.Time.now()
            tot_arrow.ns = "forces"
            tot_arrow.id = 12
            tot_arrow.type = Marker.ARROW
            tot_arrow.action = Marker.ADD
            tot_arrow.scale.x = 0.12
            tot_arrow.scale.y = 0.24
            tot_arrow.scale.z = 0.35
            tot_arrow.color.r = 0.0
            tot_arrow.color.g = 0.5
            tot_arrow.color.b = 1.0
            tot_arrow.color.a = 1.0
            tot_end = Point(self.x + self.f_total[0] * 0.8, self.y + self.f_total[1] * 0.8, 0.5)
            tot_arrow.points = [origin, tot_end]
            arr.markers.append(tot_arrow)

        self.pub_force_markers.publish(arr)

    def clear_markers(self):
        """Limpa marcadores visuais ao atingir o objetivo."""
        line = Marker()
        line.header.frame_id = "odom"
        line.ns = "target_line"
        line.id = 1
        line.action = Marker.DELETE
        self.pub_line_marker.publish(line)

        # Deletar setas
        arr = MarkerArray()
        for i in [10, 11, 12]:
            m = Marker()
            m.header.frame_id = "odom"
            m.ns = "forces"
            m.id = i
            m.action = Marker.DELETE
            arr.markers.append(m)
        self.pub_force_markers.publish(arr)

    def stop_boat(self):
        """Envia comando nulo de velocidade."""
        self.pub_cmd_vel.publish(Twist())

    def run(self):
        """Loop de controle principal (20 Hz)."""
        while not rospy.is_shutdown():
            if not self.has_pose:
                rospy.logwarn_throttle(3.0, "[Waypoint APF Nav] Aguardando telemetria (/odom)...")
                self.rate.sleep()
                continue

            if not self.has_active_goal:
                self.rate.sleep()
                continue

            # 1. Distancia euclidiana ate o waypoint
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            dist = math.hypot(dx, dy)

            # 2. Verificar se alcancou o objetivo
            if dist <= self.goal_tolerance:
                self.stop_boat()
                self.has_active_goal = False
                self.clear_markers()
                rospy.loginfo("[Waypoint APF Nav] >>> OBJETIVO ALCANCADO COM SUCESSO! (Dist: %.2fm). Barco parado.", dist)
                rospy.loginfo("[Waypoint APF Nav] Clique em '2D Nav Goal' no RViz para o proximo waypoint...")
                self.rate.sleep()
                continue

            # 3. Calcular Forcas do Campo Potencial
            self.f_att = self.compute_attractive_force()
            self.f_total = self.f_att + self.f_rep

            # 4. Escape de Minimo Local
            # Se a resultante anular por estar exatamente de frente com a boia
            total_mag = math.hypot(self.f_total[0], self.f_total[1])
            if total_mag < 0.2 and np.linalg.norm(self.f_rep) > 0.4:
                rospy.logwarn_throttle(1.0, "[Waypoint APF Nav] Minimo local detectado! Aplicando forca de contorno...")
                escape = np.array([-self.f_rep[1], self.f_rep[0]])
                self.f_total += 1.2 * (escape / np.linalg.norm(escape))

            # 5. Publicar Marcadores Visuais no RViz
            self.publish_target_line()
            self.publish_force_markers()

            # 6. Orientacao Desejada da Proa (Angulo da Forca Resultante)
            desired_yaw = math.atan2(self.f_total[1], self.f_total[0])
            yaw_error = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))

            # 7. Controle de Movimento do Barco
            # - Velocidade Angular (Guinada)
            cmd_angular_z = np.clip(self.k_p_yaw * yaw_error, -self.max_angular_speed, self.max_angular_speed)

            # - Velocidade Linear (Surge)
            # Desaceleracao suave proxima ao alvo
            if dist < self.slowdown_dist:
                base_linear = self.max_linear_speed * max(0.25, (dist / self.slowdown_dist))
            else:
                base_linear = self.max_linear_speed

            # Se estiver desalinhado (> 55 graus), prioriza giro antes de acelerar
            if abs(yaw_error) > math.radians(55.0):
                cmd_linear_x = 0.05
            else:
                cmd_linear_x = base_linear * max(0.0, math.cos(yaw_error))

            # Manobra defensiva caso boia esteja muito proxima a proa
            if self.min_front_dist < self.d_critical:
                rospy.logwarn_throttle(0.8,
                    "[Waypoint APF Nav] DESVIANDO! Boia detectada a %.2fm da proa. Reduzindo avanco!", self.min_front_dist)
                cmd_linear_x = min(cmd_linear_x, 0.18)
                cmd_angular_z = math.copysign(self.max_angular_speed, yaw_error if abs(yaw_error) > 0.05 else 1.0)

            # 8. Publicar Comando de Velocidade
            cmd = Twist()
            cmd.linear.x = float(cmd_linear_x)
            cmd.angular.z = float(cmd_angular_z)
            self.pub_cmd_vel.publish(cmd)

            rep_norm = np.linalg.norm(self.f_rep)
            rospy.loginfo_throttle(1.5,
                "[Waypoint APF Nav] Dist Alvo: %.2fm | Menor Obst: %.2fm | F_rep: %.2f | V: %.2fm/s | W: %.2frad/s",
                dist, self.min_front_dist, rep_norm, cmd_linear_x, cmd_angular_z
            )

            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = BoatWaypointAPFNav()
        node.run()
    except rospy.ROSInterruptException:
        pass
