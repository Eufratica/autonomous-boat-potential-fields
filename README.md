# 🚤 Navegação Autônoma de Embarcação com Campos Potenciais Artificiais (APF)

Projeto de navegação autônoma e desvio dinâmico de obstáculos para embarcações de superfície (**Autonomous Surface Vessels - ASVs**) utilizando **ROS Noetic**, simulador **Gazebo Classic** e algoritmo de **Campos Potenciais Artificiais (Artificial Potential Fields - APF)** com sensor LiDAR 2D e interface interativa no **RViz**.

---

## 📌 Sumário
1. [Visão Geral](#-visão-geral)
2. [Fundamentação Teórica: Campos Potenciais Artificiais](#-fundamentação-teórica-campos-potenciais-artificiais)
   - [Força Atrativa](#1-força-atrativa-vecf_att)
   - [Força Repulsiva](#2-força-repulsiva-vecf_rep)
   - [Transformação de Coordenadas (Robô $\to$ Mundo)](#3-transformação-de-coordenadas-robô-to-mundo)
   - [Força Resultante e Controle Cinemático](#4-força-resultante-e-controle-cinemático)
3. [Componentes da Simulação](#-componentes-da-simulação)
   - [Arena Aquática Fechada](#arena-aquática-fechada-42-boias)
   - [Modelo da Embarcação (Migbot 1)](#modelo-da-embarcação-migbot-1)
   - [Visualização 3D das Forças no RViz](#visualização-3d-das-forças-no-rviz)
4. [Estrutura do Repositório](#-estrutura-do-repositório)
5. [Tópicos ROS](#-tópicos-ros)
6. [Guia de Instalação e Execução](#-guia-de-instalação-e-execução)
   - [Pré-requisitos](#pré-requisitos)
   - [Execução via Docker](#execução-via-docker-recomendado)
   - [Comando de Inicialização Rápida](#comando-de-inicialização-rápida)
   - [Como Navegar Interativamente pelo RViz](#como-navegar-interativamente-pelo-rviz)
7. [Parâmetros de Ajuste do Algoritmo](#-parâmetros-de-ajuste-do-algoritmo)
8. [Desafios Técnicos Superados](#-desafios-técnicos-superados)
9. [Créditos e Referências](#-créditos-e-referências)

---

## 🌊 Visão Geral

Este projeto desenvolve uma solução completa de navegação autônoma em ambiente marinho controlado:
- **Simulação Hidrodinâmica:** Barco bimotor/multimotor simulado com forças de empuxo, arrasto e flutuabilidade da água no Gazebo.
- **Detecção com LiDAR 2D:** Varredura angular de 360° montada na proa do barco para detecção de obstáculos em tempo real.
- **Desvio Inteligente de Obstáculos:** Implementação de campos potenciais que repelem o barco de boias náuticas enquanto mantêm a atração ao ponto de destino.
- **Controle Interativo:** O usuário define o destino clicando diretamente no mapa com a ferramenta **2D Nav Goal** do RViz.
- **Telemetria e Visualização Vetorial:** Exibição em tempo real de flechas 3D representando a força atrativa (verde), repulsiva (vermelho) e a força total resultante (azul).

---

## 🧠 Fundamentação Teórica: Campos Potenciais Artificiais

O método de **Campos Potenciais Artificiais (Artificial Potential Fields - APF)**, proposto originalmente por Oussama Khatib (1986), modela o robô como uma partícula imersa em um campo de forças virtuais:
- O **ponto de destino** (Goal) gera um **poço de potencial atrativo**, puxando o robô em sua direção.
- Cada **obstáculo** detectado gera um **pico de potencial repulsivo**, empurrando o robô para longe.
- A **força resultante** determina a direção e magnitude do comando de movimento.

```
                  [ Obstáculo (Boia) ]
                            ▲
                            │  F_rep (Repulsão)
                            │
   [ Barco ] ═══════════════╬════════════════► [ 2D Nav Goal ]
             \              ║                   F_att (Atração)
              \             ▼
               \═════════► F_total (Resultante com desvio)
```

---

### 1. Força Atrativa ($\vec{F}_{att}$)

A função de energia potencial atrativa $U_{att}(q)$ em relação à distância até o alvo $d(q, q_{goal}) = \|\mathbf{p}_{goal} - \mathbf{p}_{barco}\|$ é definida como:

$$U_{att}(q) = \frac{1}{2} k_{att} \cdot d(q, q_{goal})^2$$

A força atrativa é o gradiente negativo do potencial:

$$\vec{F}_{att} = -\nabla U_{att}(q) = k_{att} \cdot (\mathbf{p}_{goal} - \mathbf{p}_{barco})$$

No nosso script, para evitar forças excessivas quando o alvo está muito distante, normalizamos o vetor de direção e modulamos por um ganho saturado:

$$\vec{u}_{att} = \frac{\mathbf{p}_{goal} - \mathbf{p}_{barco}}{\|\mathbf{p}_{goal} - \mathbf{p}_{barco}\|}$$
$$\vec{F}_{att} = k_{att} \cdot \vec{u}_{att}$$

Quando o barco se aproxima do raio de parada ($d \le 1.5\text{ m}$), a velocidade desacelera suavemente até zero.

---

### 2. Força Repulsiva ($\vec{F}_{rep}$)

Para cada feixe do LiDAR $i$ com leitura de distância $d_i$ dentro do raio de influência do obstáculo ($d_i < d_0$), calcula-se a energia potencial de repulsão:

$$U_{rep, i} = -\frac{1}{2} k_{rep} \left( \frac{1}{d_i} - \frac{1}{d_0} \right)^2$$

onde:
- $d_i$: distância medida pelo feixe $i$ do LiDAR.
- $d_0$: distância máxima de influência dos obstáculos (ex: $4.5\text{ metros}$). Obstáculos além de $d_0$ não afetam a trajetória.
- $k_{rep}$: ganho de repulsão (peso de afastamento do obstáculo).

As componentes da força de repulsão no referencial local do robô são:

$$F_{rep, x}^{local} = \sum_{i} \cos(\theta_i) \cdot U_{rep, i}$$
$$F_{rep, y}^{local} = \sum_{i} \sin(\theta_i) \cdot U_{rep, i}$$

onde $\theta_i = \theta_{min} + i \cdot \Delta\theta$ é o ângulo do feixe no referencial do LiDAR.

---

### 3. Transformação de Coordenadas (Robô $\to$ Mundo)

Como as leituras do LiDAR são feitas no referencial local da embarcação (`base_link`), a força repulsiva precisa ser rotacionada para o referencial inercial do mundo/odom através do ângulo de proa (*heading* atual $\psi$ do barco obtido da odometria):

$$\begin{bmatrix} F_{rep, x}^{world} \\ F_{rep, y}^{world} \end{bmatrix} = \begin{bmatrix} \cos(\psi) & -\sin(\psi) \\ \sin(\psi) & \cos(\psi) \end{bmatrix} \begin{bmatrix} F_{rep, x}^{local} \\ F_{rep, y}^{local} \end{bmatrix}$$

---

### 4. Força Resultante e Controle Cinemático

A força total atuante sobre a embarcação no plano 2D é a soma vetorial:

$$\vec{F}_{total} = \vec{F}_{att} + \vec{F}_{rep}^{world}$$

A partir do vetor $\vec{F}_{total} = [F_x, F_y]^T$, determinamos o ângulo de navegação desejado $\theta_{des}$:

$$\theta_{des} = \text{atan2}(F_y, F_x)$$

O erro angular relativo à proa atual $\psi$ do barco é obtido normalizado em $[-\pi, \pi]$:

$$e_\theta = \text{atan2}(\sin(\theta_{des} - \psi), \cos(\theta_{des} - \psi))$$

Os comandos de controle enviados ao tópico `/cmd_vel` são gerados por:

1. **Velocidade Angular ($\omega$):**
   $$\omega = k_\omega \cdot e_\theta$$

2. **Velocidade Linear ($v$):**
   $$v = v_{base} \cdot \max(0, \cos(e_\theta)) \cdot f_{frenagem}$$
   *(Se o barco estiver muito desalinhado em relação à direção da força, ele reduz a velocidade linear para manobrar no próprio eixo e evitar colisões).*

---

## 🛠️ Componentes da Simulação

### Arena Aquática Fechada (42 Boias)
O arquivo [`closed_aquatic_arena.world`](xasv_sim/worlds/closed_aquatic_arena.world) posiciona **42 boias marítimas cilíndricas com esferas sinalizadoras** dispostas estrategicamente:
- **Boias Vermelhas (`Gazebo/Red`):** Balizamento de bombordo e marcação de perigo à esquerda.
- **Boias Verdes (`Gazebo/Green`):** Balizamento de boreste e canais de passagem à direita.
- **Boias Amarelas (`Gazebo/Yellow`):** Zonas de aviso e slalom central para validação de desvio contínuo.

### Modelo da Embarcação (Migbot 1)
- Casco catamarã/monocasco com sustentação hidrostática no Gazebo.
- 6 hélices com modelos cinemáticos.
- Sensor **LiDAR planar 2D** calibrado a uma altura ideal sobre a linha d'água para varredura contínua de 360°.
- Sensor **IMU** com link de transformada (`imu_link`) acoplado e fixado rigidamente à fuselagem.

### Visualização 3D das Forças no RViz
Para depuração visual e entendimento da física em tempo real, o nó publica marcadores do tipo `visualization_msgs/Marker`:
| Cor | Vetor | Significado |
| :---: | :---: | :--- |
| 🟢 **Verde** | $\vec{F}_{att}$ | Força Atrativa (aponta para o 2D Nav Goal) |
| 🔴 **Vermelho** | $\vec{F}_{rep}$ | Força Repulsiva (repulsão proporcional das boias) |
| 🔵 **Azul** | $\vec{F}_{total}$ | Força Resultante (para onde o barco realmente navega) |
| 🟡 **Amarelo** | `Goal Marker` | Marcador esférico sobre o ponto de chegada |
| 🔴 **Linha Vermelha** | `Path` | Rastro do histórico da trajetória percorrida pelo barco |

---

## 📁 Estrutura do Repositório

```text
autonomous-boat-potential-fields/
├── robots/
│   ├── migbot_description/urdf/
│   │   ├── migbot1.urdf.xacro          # Correção: imu_joint corrigido para 'fixed'
│   │   ├── migbot1_ros.urdf.xacro      # Integração com ROS Controllers
│   │   └── migbot1_ardupilot.urdf.xacro
│   └── migbot_gazebo/urdf/
│       ├── components/planar_lidar_mount.xacro # Suporte mecânico e plugin do LiDAR
│       └── migbot_gazebo.urdf.xacro    # Inclusão dos sensores e dinâmica
├── xasv_sim/
│   ├── launch/
│   │   └── boat_aquatic_arena.launch   # Launch principal unificado (Mundo + Barco + RViz + APF)
│   ├── rviz/
│   │   └── waypoint_nav.rviz           # Perfil configurado do RViz com TF, LiDAR e Vetores
│   ├── scripts/
│   │   └── boat_waypoint_nav.py        # Algoritmo de APF com 2D Nav Goal e controle de velocidade
│   └── worlds/
│       └── closed_aquatic_arena.world  # Arena aquática com 42 boias de balizamento
└── README.md
```

---

## 📡 Tópicos ROS

| Tópico | Tipo de Mensagem | Descrição |
| :--- | :--- | :--- |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Destino recebido do clique da ferramenta **2D Nav Goal** no RViz |
| `/scan` | `sensor_msgs/LaserScan` | Leituras de distância dos feixes do LiDAR 2D |
| `/migbot1/odom` | `nav_msgs/Odometry` | Odometria e atitude (proa/heading) do barco |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidade linear (`linear.x`) e angular (`angular.z`) |
| `/apf/att_marker` | `visualization_msgs/Marker` | Seta 3D da Força Atrativa no RViz |
| `/apf/rep_marker` | `visualization_msgs/Marker` | Seta 3D da Força Repulsiva no RViz |
| `/apf/total_marker` | `visualization_msgs/Marker` | Seta 3D da Força Resultante no RViz |
| `/apf/path` | `nav_msgs/Path` | Histórico da trajetória percorrida |

---

## 🚀 Guia de Instalação e Execução

### Pré-requisitos
- Ubuntu 20.04 LTS
- ROS Noetic Desktop Full
- Gazebo 11

### Execução via Docker (Recomendado)
Se você estiver utilizando o container Docker do projeto (`xasv_gpu`):

```bash
# 1. No terminal do seu computador (fora do container):
xhost +local:
docker start xasv_gpu
docker exec -it xasv_gpu bash
```

### Comando de Inicialização Rápida
Dentro do ambiente ROS, execute o launch unificado:

```bash
roslaunch xasv_sim boat_aquatic_arena.launch gui:=true rviz:=true
```

Esse comando inicializa automaticamente:
1. O simulador **Gazebo** com a arena fechada e as 42 boias.
2. O modelo da embarcação com empuxo e física aquática.
3. O nó de controle `joint_state_publisher` publicando o giro das 6 hélices e a pose da IMU.
4. O nó de campos potenciais `boat_waypoint_nav.py`.
5. O **RViz** com a visualização completa já pré-configurada.

---

### Como Navegar Interativamente pelo RViz

1. Com a simulação rodando, olhe para a barra superior do **RViz**.
2. Clique na ferramenta **`2D Nav Goal`** (ou pressione a tecla `G`).
3. Clique em qualquer ponto da água do mapa e arraste a seta apontando a orientação desejada.
4. **Observe o barco em ação:**
   - A seta verde ($ec{F}_{att}$) se estenderá apontando para o seu ponto.
   - Assim que o barco se aproximar de qualquer boia, a seta vermelha ($ec{F}_{rep}$) crescerá empurrando-o para a direção livre.
   - O barco contornará as boias de forma suave e chegará com precisão no alvo, reduzindo a velocidade até parar.

---

## ⚙️ Parâmetros de Ajuste do Algoritmo

No script [`boat_waypoint_nav.py`](xasv_sim/scripts/boat_waypoint_nav.py), os parâmetros fundamentais de navegação podem ser facilmente afinados:

```python
self.d0 = 4.5           # Raio de influência dos obstáculos em metros (distância que o barco começa a desviar)
self.k_rep = 2.8        # Ganho de repulsão (quanto maior, mais longe das boias o barco passará)
self.k_att = 1.0        # Ganho da força atrativa em direção ao ponto de chegada
self.max_linear_speed = 1.5   # Velocidade linear máxima do barco (m/s)
self.max_angular_speed = 1.2  # Velocidade máxima de giro (rad/s)
self.goal_tolerance = 1.5     # Raio de aceitação de chegada ao destino (metros)
```

---

## 🔧 Desafios Técnicos Superados

Durante o desenvolvimento deste sistema, foram solucionadas as seguintes questões críticas de simulação robótica:

1. **Correção da Árvore de Transformadas (TF Tree):**
   - No URDF original, a junta `imu_joint` estava configurada como `type="revolute"` sem um controlador de junta ativo, o que quebrava a publicação do frame `imu_link` no TF. Foi alterada para `type="fixed"`, estabilizando a telemetria inercial.
   - As 6 hélices (`Helice_1` a `Helice_6`) foram integradas via `joint_state_publisher` mapeado no tópico `/migbot1/joint_states`.
2. **Compatibilidade Gráfica no Docker:**
   - Correção de permissões de GLX e configuração de fallback via software/Mesa para ambientes conteinerizados sem conflitos de socket X11.
3. **Bloqueio de Inicialização Offline do Gazebo:**
   - Reconfiguração do `model_database.config` para evitar travamento de 3 minutos no boot do Gazebo ao tentar acessar servidores remotos desativados.
4. **Estabilidade Numérica no APF:**
   - Introdução de limiar mínimo no denominador da força repulsiva para prevenir singularidades numéricas ($\frac{1}{d} \to \infty$) quando feixes tocam obstáculos a distâncias ultra-curtas.

---

## 👥 Créditos e Referências

- **Autor:** [Eufratica](https://github.com/Eufratica)
- **Repositório Oficial:** [autonomous-boat-potential-fields](https://github.com/Eufratica/autonomous-boat-potential-fields)
- **Referência Teórica:**
  - Khatib, O. (1986). *Real-Time Obstacle Avoidance for Manipulators and Mobile Robots*. IEEE International Conference on Robotics and Automation.
  - Implementações de referência em APF do ecossistema `iq_gnc` para ROS.
- **Agradecimentos:** Ao laboratório e professor pela disponibilização da base original de modelos hidrodinâmicos do ASV (`xasv-sim`).

---
<p align="center">
  <b>Desenvolvido com 💙 para robótica autônoma móvel e superfícies aquáticas</b>
</p>
