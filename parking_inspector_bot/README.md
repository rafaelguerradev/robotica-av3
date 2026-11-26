# Robô Móvel Autônomo para Monitoramento de Vagas de Estacionamento (`parking_inspector_bot`)

Pacote completo de simulação em **ROS2 + Gazebo + RViz2** de um robô diferencial autônomo projetado para inspecionar periodicamente um estacionamento, classificar cada vaga como **LIVRE** ou **OCUPADA** por meio de **Sensor Ultrassônico Lateral (Opção C)** e atualizar em tempo real um **Sistema Central de Monitoramento** (painel no terminal, arquivo JSON e marcadores 3D no RViz2).

---

## 1. Geometria do Cenário Simulado

O mundo `worlds/parking_lot.world` modela o estacionamento com precisão métrica:

| Elemento | Especificação | Coordenadas no Mundo (`odom` / Gazebo) |
| :--- | :--- | :--- |
| **Estação Base (Início/Retorno)** | Plataforma visual verde de `1.2 m x 1.2 m` | Centro em `(X = 0.0, Y = 0.0)` |
| **Corredor de Circulação** | Faixa livre central de `19.0 m` | Eixo `Y = 0.0 m`, de `X = -1.0 m` a `X = 18.0 m` |
| **5 Vagas Delimitadas** | `3.0 m` (largura em `X`) x `5.0 m` (profundidade em `Y`) | `Y` de `-1.0 m` a `-6.0 m` (centro em `Y = -3.5 m`) |
| **Vaga 1 (OCUPADA)** | Carro Azul/Prata (`2.2 m x 4.2 m x 1.4 m`) | Centro em `(X = 3.0, Y = -3.5)` |
| **Vaga 2 (LIVRE)** | Sem veículo | Centro em `(X = 6.0, Y = -3.5)` |
| **Vaga 3 (OCUPADA)** | Carro Vermelho (`2.2 m x 4.2 m x 1.4 m`) | Centro em `(X = 9.0, Y = -3.5)` |
| **Vaga 4 (OCUPADA)** | Carro Preto/Cinza (`2.2 m x 4.2 m x 1.4 m`) | Centro em `(X = 12.0, Y = -3.5)` |
| **Vaga 5 (LIVRE)** | Sem veículo | Centro em `(X = 15.0, Y = -3.5)` |
| **Parede de Fundo** | Comprimento `19.5 m`, altura `1.5 m` | Face interna em `Y = -6.20 m` |

### Princípio Físico da Classificação Ultrassônica (Opção C)
* O sensor ultrassônico lateral direito (`ultrasonic_right_link`) fica posicionado a `Y = -0.18 m` quando o robô percorre o eixo `Y = 0.0 m` alinhado em `yaw = 0.0 rad`.
* **Vaga Ocupada:** A frente do veículo (`4.2 m` de comprimento centrado em `Y = -3.5 m`) localiza-se em `Y = -1.40 m`, gerando uma distância média medida $\mu \approx 1.22\text{ m}$.
* **Vaga Livre:** O feixe acústico atravessa toda a vaga de `5.0 m` e reflete na parede de fundo em `Y = -6.20 m`, gerando uma distância média medida $\mu \approx 6.02\text{ m}$.
* **Regra de Decisão:** Com o limiar calibrado em `occupancy_threshold_m = 4.2 m`:
  * $\mu < 4.2\text{ m} \implies \text{OCUPADA}$
  * $\mu \ge 4.2\text{ m} \implies \text{LIVRE}$

---

## 2. Estrutura do Pacote

```text
parking_inspector_bot/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── parking_inspector_bot
├── urdf/
│   └── parking_bot.urdf.xacro
├── worlds/
│   └── parking_lot.world
├── rviz/
│   └── parking_monitor.rviz
├── launch/
│   ├── sim_complete.launch.py
│   └── physical_prototype.launch.py
└── parking_inspector_bot/
    ├── __init__.py
    ├── patrol_navigator_node.py
    ├── ultrasonic_detector_node.py
    ├── central_monitor_node.py
    └── hardware_bridge_node.py
```

---

## 3. Pré-requisitos e Instalação de Dependências

Em um ambiente Ubuntu com **ROS2 Humble** (ou Jazzy com Gazebo Classic / `gazebo_ros_pkgs`), instale os pacotes necessários:

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-gazebo-ros-pkgs \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-tf2-ros
```

---

## 4. Compilação e Execução da Simulação

### 4.1. Compilar o Workspace
A partir da raiz do seu workspace ROS2 (diretório que contém a pasta `parking_inspector_bot`, ou `ros2_ws/` caso esteja dentro de `src/`):

```bash
colcon build --symlink-install --packages-select parking_inspector_bot
source install/setup.bash
```

### 4.2. Iniciar a Simulação Completa
Execute o launch integrado para abrir simultaneamente o **Gazebo**, o **RViz2** configurado e os **3 nós autônomos**:

```bash
ros2 launch parking_inspector_bot sim_complete.launch.py
```

---

## 5. Arquitetura de Nós, Tópicos e Frames TF

### 5.1. Nós ROS2 Executáveis
1. **`patrol_navigator` (`patrol_navigator_node.py`):**
   * Máquina de Estados Finitos (`START_DELAY` $\rightarrow$ `MOVING_TO_SPOT` $\rightarrow$ `INSPECTING_SPOT` $\rightarrow$ `RETURNING_TO_BASE` $\rightarrow$ `WAIT_NEXT_CYCLE`).
   * Controlador Proporcional (P) linear e angular sobre `/odom` para visitar `X = [3.0, 6.0, 9.0, 12.0, 15.0]` m mantendo `Y = 0.0` m e retornar automaticamente para `(0.0, 0.0)` m a cada ciclo periódico (`15.0 s`).
   * Sistema de anticolisão ativo: interrompe imediatamente o movimento caso `/ultrasonic/front` meça distância inferior a `0.40 m`.
2. **`ultrasonic_detector` (`ultrasonic_detector_node.py`):**
   * Mantém buffer circular (`maxlen=15`) das leituras de `/ultrasonic/right`.
   * Ao receber o ID da vaga em `/inspection_trigger`, aguarda `1.5 s` de coleta estacionária pura, calcula média ($\mu$) e desvio padrão ($\sigma$) e publica o diagnóstico estruturado em JSON no tópico `/spot_status_update`.
3. **`central_monitor` (`central_monitor_node.py`):**
   * Renderiza tabela formatada em tempo real no terminal com o estado das 5 vagas, contagem de livres/ocupadas e pose atual do robô.
   * Persiste continuamente o relatório consolidado em `/tmp/parking_central_status.json`.
   * Publica marcadores 3D (`visualization_msgs/MarkerArray`) em `/parking_markers` para visualização imediata no RViz2 (cubos de `2.8 m x 4.8 m x 0.08 m` em cinza, verde ou vermelho + rótulos flutuantes a `Z = 1.8 m`).

### 5.2. Mapa de Tópicos ROS2

| Tópico | Tipo de Mensagem | Publicador | Assinante(s) | Descrição |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `patrol_navigator` | `gazebo_ros_diff_drive` | Comandos de velocidade linear ($v_x$) e angular ($\omega_z$) |
| `/odom` | `nav_msgs/msg/Odometry` | `gazebo_ros_diff_drive` | `patrol_navigator`, `central_monitor` | Pose estimada e velocidade do robô no referencial `odom` |
| `/ultrasonic/right` | `sensor_msgs/msg/Range` | `gazebo_ros_ray_sensor` | `ultrasonic_detector`, `rviz2` | Cone ultrassônico direito (`max_range = 7.0 m`) voltado às vagas |
| `/ultrasonic/front` | `sensor_msgs/msg/Range` | `gazebo_ros_ray_sensor` | `patrol_navigator`, `rviz2` | Cone ultrassônico frontal (`max_range = 5.0 m`) para anticolisão |
| `/inspection_trigger` | `std_msgs/msg/Int32` | `patrol_navigator` | `ultrasonic_detector` | ID da vaga (`1` a `5`) disparado quando o robô estaciona alinhado |
| `/spot_status_update` | `std_msgs/msg/String` | `ultrasonic_detector` | `central_monitor` | JSON com `spot_id`, `status`, `mean_distance_m`, `std_dev_m` e `samples` |
| `/patrol_state` | `std_msgs/msg/String` | `patrol_navigator` | `central_monitor` | Estado atual da máquina de estados da ronda |
| `/parking_markers` | `visualization_msgs/msg/MarkerArray` | `central_monitor` | `rviz2` | Cubos 3D coloridos das vagas e textos flutuantes no RViz2 |

### 5.3. Árvore de Transformações (`TF`)
* `odom` $\rightarrow$ `base_footprint` (publicada dinamicamente por `libgazebo_ros_diff_drive.so`)
* `base_footprint` $\rightarrow$ `base_link` (`xyz="0 0 0.135"`, junta fixa via `robot_state_publisher`)
* `base_link` $\rightarrow$ `left_wheel` e `right_wheel` (juntas `continuous` publicadas por `libgazebo_ros_diff_drive.so`)
* `base_link` $\rightarrow$ `caster_wheel` (`xyz="0.18 0 -0.09"`, esfera de apoio sem atrito)
* `base_link` $\rightarrow$ `ultrasonic_right_link` (`xyz="0.0 -0.18 0.10"`, `rpy="0 0 -1.570796"`)
* `base_link` $\rightarrow$ `ultrasonic_front_link` (`xyz="0.25 0.0 0.10"`, `rpy="0 0 0"`)

---

## 6. Comandos Úteis de Inspeção e Diagnóstico

```bash
# Acompanhar atualizacoes JSON de classificacao de cada vaga
ros2 topic echo /spot_status_update

# Inspecionar leituras em tempo real do sensor ultrassonico lateral
ros2 topic echo /ultrasonic/right

# Consultar o arquivo JSON consolidado do Sistema Central
cat /tmp/parking_central_status.json
```

---

## 7. Operação com o Protótipo Físico (ESP32-WROOM-32)

Para integrar o protótipo real (pasta `../firmware_prototipo/`) ao **Sistema Central de Monitoramento** e ao **RViz2** via porta serial USB (`115200` baud):

```bash
sudo apt install -y python3-serial
colcon build --symlink-install --packages-select parking_inspector_bot
source install/setup.bash

# Inicia a ponte serial (/dev/ttyUSB0), o painel central e o RViz2 ao vivo
ros2 launch parking_inspector_bot physical_prototype.launch.py serial_port:=/dev/ttyUSB0

# Opcional: disparar inicio da missao ou ajustar dimensoes das vagas fisicas via ROS2
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'START'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'SET_LARGURA:1.60'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'SET_COMPRIMENTO:0.68'}"
```
