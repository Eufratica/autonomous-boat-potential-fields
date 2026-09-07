#!/usr/bin/env python3
import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan  # Ou o tipo de mensagem do seu sensor de obstáculo
from nav_msgs.msg import Odometry      # Ou dados de posição/estado do MAVROS

# --- 1. REDES NEURAIS DO TD3 (Actor & Critic) ---
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.layer1 = nn.Linear(state_dim, 256)
        self.layer2 = nn.Linear(256, 256)
        self.layer3 = nn.Linear(256, action_dim)
        self.max_action = max_action

    def forward(self, state):
        a = torch.relu(self.layer1(state))
        a = torch.relu(self.layer2(a))
        return self.max_action * torch.tanh(self.layer3(a))

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        # Q1 architecture
        self.l1 = nn.Linear(state_dim + action_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, 1)
        # Q2 architecture
        self.l4 = nn.Linear(state_dim + action_dim, 256)
        self.l5 = nn.Linear(256, 256)
        self.l6 = nn.Linear(256, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        q1 = torch.relu(self.l1(sa))
        q1 = torch.relu(self.l2(q1))
        q1 = self.l3(q1)

        q2 = torch.relu(self.l4(sa))
        q2 = torch.relu(self.l5(q2))
        q2 = self.l6(q2)
        return q1, q2

# --- 2. AMBIENTE ROS PARA O BARCO ---
class BoatEnv:
    def __init__(self):
        rospy.init_node('boat_td3_trainer', anonymous=True)
        
        # Publicador de comandos de velocidade para o MAVROS (Modo GUIDED)
        self.cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        
        # Variáveis de Estado
        self.min_front = 5.0
        self.min_left = 5.0
        self.min_right = 5.0
        self.speed = 0.0
        self.dist_to_wp = 10.0
        self.bearing_to_wp = 0.0
        
        self.collided = False
        self.goal_reached = False

        # Assinaturas de Sensores (Ajuste os tópicos conforme o seu Gazebo/MAVROS)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)

    def laser_callback(self, msg):
        # Exemplo simples dividindo o LaserScan em frentes, esquerda e direita
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        
        n = len(ranges)
        if n > 0:
            self.min_front = np.min(ranges[int(n*0.4):int(n*0.6)])
            self.min_left = np.min(ranges[int(n*0.6):])
            self.min_right = np.min(ranges[:int(n*0.4)])
            
            # Condição simples de colisão
            if self.min_front < 0.8 or self.min_left < 0.5 or self.min_right < 0.5:
                self.collided = True

    def odom_callback(self, msg):
        # Velocidade linear atual
        self.speed = msg.twist.twist.linear.x
        # (Aqui você pode calcular a distância e o bearing real até o waypoint atual)

    def get_state(self):
        # Estado retornado para a rede neural
        return np.array([
            self.min_front, 
            self.min_left, 
            self.min_right, 
            self.speed, 
            self.dist_to_wp, 
            self.bearing_to_wp
        ], dtype=np.float32)

    def step(self, action):
        # action[0] = Aceleração linear (Throttle) [-1 a 1]
        # action[1] = Giro / Leme (Steering) [-1 a 1]
        
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = float(action[0]) * 2.0  # Escala para até 2 m/s
        twist.twist.angular.z = float(action[1]) * 1.0 # Escala para guinada
        self.cmd_pub.publish(twist)
        
        rospy.sleep(0.1) # Taxa de atualização do ciclo (10Hz)
        
        next_state = self.get_state()
        
        # --- CÁLCULO DA RECOMPENSA (Reward Function) ---
        reward = 0.0
        done = False
        
        # 1. Recompensa por avançar (exemplo baseado na distância ao waypoint)
        reward += self.speed * 0.5
        
        # 2. Punição severa se estiver muito perto de obstáculos ou colidir
        if self.collided or self.min_front < 1.0:
            reward -= 50.0
            done = True
            
        # 3. Punição leve por ficar muito perto das laterais
        if self.min_left < 0.8 or self.min_right < 0.8:
            reward -= 2.0

        return next_state, reward, done

    def reset(self):
        self.collided = False
        self.goal_reached = False
        # Aqui você pode chamar um serviço do Gazebo para resetar a posição do barco se desejar
        rospy.sleep(0.5)
        return self.get_state()

# --- 3. LOOP PRINCIPAL DE TREINAMENTO ---
if __name__ == '__main__':
    try:
        env = BoatEnv()
        state_dim = 6
        action_dim = 2
        max_action = 1.0

        # Inicializa o Agente Actor e Critic (simplificado)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        actor = Actor(state_dim, action_dim, max_action).to(device)
        critic = Critic(state_dim, action_dim).to(device)
        
        actor_optimizer = optim.Adam(actor.parameters(), lr=3e-4)
        critic_optimizer = optim.Adam(critic.parameters(), lr=3e-4)

        rospy.loginfo("[TD3-TRAIN] Ambiente de Treinamento iniciado. Pronto para rodar episódios!")

        num_episodes = 500
        for episode in range(num_episodes):
            state = env.reset()
            episode_reward = 0
            done = False

            while not done and not rospy.is_shutdown():
                # Seleção de ação com um pouco de ruído para exploração
                state_tensor = torch.FloatTensor(state.reshape(1, -1)).to(device)
                action = actor(state_tensor).cpu().data.numpy().flatten()
                action = np.clip(action + np.random.normal(0, 0.1, size=action_dim), -max_action, max_action)

                next_state, reward, done = env.step(action)
                
                episode_reward += reward
                state = next_state

                if done:
                    break

            rospy.loginfo(f"Episódio {episode+1}/{num_episodes} finalizado. Recompensa Total: {episode_reward:.2f}")

    except rospy.ROSInterruptException:
        pass