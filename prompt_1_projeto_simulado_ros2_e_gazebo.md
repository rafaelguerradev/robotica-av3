# PROMPT DE ENGENHARIA (PARTE 1): SIMULAÇÃO COMPLETA DO ROBÔ AUTÔNOMO DE ESTACIONAMENTO (ROS2 + GAZEBO + RVIZ2)

Atue como um **Engenheiro Sênior de Robótica Autônoma (ROS2)**.
Sua tarefa é criar no meu workspace do VS Code, do zero e **sem usar nenhum código incompleto, resumo ou placeholder (`TODO`, `pass`, `...`)**, o pacote completo de **simulação** de um **Robô Móvel Autônomo para Monitoramento de Vagas de Estacionamento utilizando Sensor Ultrassônico (Opção C)**.

---

## 1. CONTEXTO E REQUISITOS OBRIGATÓRIOS DA SIMULAÇÃO

* **Objetivo:** Projetar e simular um robô móvel autônomo capaz de percorrer periodicamente um estacionamento, identificar vagas ocupadas e desocupadas utilizando **ROS2** e atualizar um **Sistema Central de Monitoramento** em tempo real.
* **Dimensões Exatas do Cenário Simulado (OBRIGATÓRIO):**
  * **Quantidade de vagas:** Pelo menos **5 vagas** delimitadas lado a lado.
  * **Dimensão de cada vaga:** **$3.0\text{ metros}$ de largura** (ao longo do eixo $X$ do corredor) por **$5.0\text{ metros}$ de comprimento/profundidade** (estendendo-se lateralmente no eixo $-Y$).
  * **Dimensão dos veículos simulados:** **$2.2\text{ metros}$ de largura** por **$4.2\text{ metros}$ de comprimento** (e $1.4\text{ metros}$ de altura).
  * **Ocupação inicial:** Veículos posicionados nas **Vagas 1, 3 e 4** (OCUPADAS); **Vagas 2 e 5** vazias (LIVRES).
  * **Corredor e Estação:** Corredor livre para deslocamento ao longo de $Y = 0.0\text{ m}$ e Estação de Início e Retorno demarcada em $(X = 0.0, Y = 0.0)$.
* **Requisitos do Robô:**
  * Modelo criado em **URDF/Xacro** contendo chassi, rodas, sensores e frames TF necessários.
  * **Sensoriamento (Opção C - Sensor Ultrassônico):** Analisar a **distância média** dentro da área de cada vaga para classificá-la como **LIVRE** ou **OCUPADA**.
  * **Atuadores:** Controle de velocidade linear ($v_x$) e velocidade angular ($\omega_z$) via tópico `/cmd_vel`.
* **Ferramentas:** ROS2 (Humble/Jazzy), Gazebo, RViz2, URDF/Xacro e Python (`rclpy`).

---

## 2. ARQUITETURA DE DIRETÓRIOS A SER CRIADA

Crie o pacote ROS2 Python (`ament_python`) chamado `parking_inspector_bot` com a seguinte estrutura exata:

~~~text
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
│   └── sim_complete.launch.py
└── parking_inspector_bot/
    ├── __init__.py
    ├── patrol_navigator_node.py
    ├── ultrasonic_detector_node.py
    └── central_monitor_node.py
~~~

---

## 3. ESPECIFICAÇÃO TÉCNICA DETALHADA DE CADA ARQUIVO

### 3.1. Configuração do Pacote (`package.xml`, `setup.py`, `setup.cfg`, `resource/parking_inspector_bot`)
1. **`package.xml`:** Formato 3, com dependências para `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `tf2_ros`, `xacro`, `robot_state_publisher`, `joint_state_publisher` e `gazebo_ros`.
2. **`setup.py`:**
   * Utilize `os.path.join` e `glob` para instalar todos os arquivos das pastas `launch/`, `urdf/`, `worlds/` e `rviz/` em `share/parking_inspector_bot/`.
   * Registre os `entry_points` (`console_scripts`):
     * `patrol_navigator = parking_inspector_bot.patrol_navigator_node:main`
     * `ultrasonic_detector = parking_inspector_bot.ultrasonic_detector_node:main`
     * `central_monitor = parking_inspector_bot.central_monitor_node:main`
3. **`setup.cfg`:** Configure `script_dir=$base/lib/parking_inspector_bot` e `install_scripts=$base/lib/parking_inspector_bot`.
4. **`resource/parking_inspector_bot`:** Arquivo marcador vazio.

---

### 3.2. Parte 1 – Modelagem do Robô (`urdf/parking_bot.urdf.xacro`)
Crie um modelo Xacro completo, parametrizado e fisicamente estável contendo:
1. **Macros de Inércia:** Macros Xacro para `box_inertia`, `cylinder_inertia` e `sphere_inertia`.
2. **Estrutura Mecânica:**
   * `base_footprint` conectado ao `base_link`.
   * `base_link` (Chassi): Caixa de $0.50 \times 0.36 \times 0.18\text{ m}$, massa de $3.0\text{ kg}$, cor azul escuro.
   * `left_wheel` e `right_wheel`: Cilindros de raio $0.09\text{ m}$ e largura $0.045\text{ m}$, massa $0.4\text{ kg}$, conectados por juntas `continuous` com eixo de rotação em $Y$ (`0 1 0`).
   * `caster_wheel`: Esfera frontal de apoio com raio $0.045\text{ m}$, junta `fixed` e parâmetros de atrito zero no Gazebo (`mu1="0.001"`, `mu2="0.001"`) para garantir giro suave.
3. **Sensores Ultrassônicos e Frames TF:**
   * `ultrasonic_right_link` e `ultrasonic_right_joint`: Posicionado na lateral direita do chassi (`xyz="0.0 -0.18 0.10"`), orientado a $-90^\circ$ no eixo $Z$ (`rpy="0 0 -1.570796"`), apontando diretamente para dentro das vagas à direita do corredor.
   * `ultrasonic_front_link` e `ultrasonic_front_joint`: Posicionado na frente do chassi (`xyz="0.25 0.0 0.10"`), orientado para frente (`rpy="0 0 0"`), para prevenção de colisões no corredor.
4. **Plugins do Gazebo:**
   * **Tração Diferencial (`libgazebo_ros_diff_drive.so`):** Assina `/cmd_vel`, publica `/odom` e publica a árvore TF (`odom` $\rightarrow$ `base_footprint` e rodas), com `wheel_separation` = $0.405\text{ m}$ e `wheel_diameter` = $0.18\text{ m}$.
   * **Sensor Ultrassônico Direito (`libgazebo_ros_ray_sensor.so`):** Associado ao `ultrasonic_right_link`, configurado com `<radiation_type>ultrasound</radiation_type>`, `<output_type>sensor_msgs/Range</output_type>`, `min_range = 0.05`, **`max_range = 7.0`** (essencial para cobrir toda a profundidade de $5.0\text{ m}$ da vaga até a parede de fundo), cone horizontal de 5 amostras entre $-0.10\text{ rad}$ e $+0.10\text{ rad}$, publicando em `/ultrasonic/right` com `frame_name` = `ultrasonic_right_link`.
   * **Sensor Ultrassônico Frontal (`libgazebo_ros_ray_sensor.so`):** Associado ao `ultrasonic_front_link`, publicando `sensor_msgs/Range` em `/ultrasonic/front` (`max_range = 5.0`) com `frame_name` = `ultrasonic_front_link`.

---

### 3.3. Parte 2 – Construção do Ambiente (`worlds/parking_lot.world`)
Crie um arquivo SDF (`.world`) completo para o Gazebo respeitando rigorosamente a geometria de vagas de **$3.0\text{ m} \times 5.0\text{ m}$** e carros de **$2.2\text{ m} \times 4.2\text{ m}$**:

1. **Iluminação e Piso:** Luz solar (`sun`) e piso asfaltado/plano (`ground_plane`).
2. **Estação de Início e Retorno:** Demarcação visual verde no chão (sem colisão) centrada em $(X = 0.0, Y = 0.0)$, tamanho $1.2 \times 1.2\text{ m}$.
3. **Corredor de Circulação:** Faixa central livre ao longo do eixo $X$, de $X = -1.0\text{ m}$ até $X = 18.0\text{ m}$, centrada em $Y = 0.0\text{ m}$ (com entrada das vagas começando em $Y = -1.0\text{ m}$).
4. **5 Vagas Delimitadas ($3.0\text{ m}$ de largura $\times$ $5.0\text{ m}$ de comprimento):**
   * Cada vaga ocupa $3.0\text{ m}$ no eixo $X$ e vai de $Y = -1.0\text{ m}$ até $Y = -6.0\text{ m}$ no eixo $Y$ (centro de todas as vagas em $Y = -3.5\text{ m}$).
   * Pinte linhas brancas no piso (`<visual>` sem colisão, espessura $0.10\text{ m}$) delimitando as bordas de cada vaga nos limites $X \in \{1.5, 4.5, 7.5, 10.5, 13.5, 16.5\}$ e fundo em $Y = -6.0\text{ m}$:
     * **Vaga 1:** $X \in [1.5, 4.5]$, $Y \in [-1.0, -6.0]$ $\rightarrow$ **Centro em $(X = 3.0, Y = -3.5)$**
     * **Vaga 2:** $X \in [4.5, 7.5]$, $Y \in [-1.0, -6.0]$ $\rightarrow$ **Centro em $(X = 6.0, Y = -3.5)$**
     * **Vaga 3:** $X \in [7.5, 10.5]$, $Y \in [-1.0, -6.0]$ $\rightarrow$ **Centro em $(X = 9.0, Y = -3.5)$**
     * **Vaga 4:** $X \in [10.5, 13.5]$, $Y \in [-1.0, -6.0]$ $\rightarrow$ **Centro em $(X = 12.0, Y = -3.5)$**
     * **Vaga 5:** $X \in [13.5, 16.5]$, $Y \in [-1.0, -6.0]$ $\rightarrow$ **Centro em $(X = 15.0, Y = -3.5)$**
5. **Paredes Limítrofes do Estacionamento:**
   * Parede de fundo atrás das vagas em $Y = -6.2\text{ m}$ (comprimento $19.5\text{ m}$, altura $1.5\text{ m}$). Assim, quando uma vaga estiver livre, o sensor ultrassônico em $Y = -0.18\text{ m}$ medirá aproximadamente $6.02\text{ m}$.
   * Parede oposta do corredor em $Y = +1.8\text{ m}$ e paredes laterais fechando o perímetro em $X = -1.5\text{ m}$ e $X = 18.0\text{ m}$.
6. **Veículos Simulados Estacionados ($2.2\text{ m}$ de largura $\times$ $4.2\text{ m}$ de comprimento):**
   * Modele carros estáticos compostos por blocos estilizados (carroceria inferior de $2.2\text{ m}$ no eixo $X$ $\times$ $4.2\text{ m}$ no eixo $Y$ $\times$ $0.75\text{ m}$ de altura + cabine superior de $1.9\text{ m} \times 2.4\text{ m} \times 0.65\text{ m}$ + 4 rodas, altura total $\approx 1.4\text{ m}$, com `<collision>` e `<visual>` ativos).
   * Posicione os veículos centralizados nas vagas ocupadas em $Y = -3.5\text{ m}$ (assim, a frente do carro fica em $Y = -1.4\text{ m}$, gerando uma leitura ultrassônica de $\approx 1.22\text{ m}$ quando a vaga está ocupada):
     * **Carro 1 (Azul/Prata):** Centralizado na **Vaga 1** em $(X = 3.0, Y = -3.5)$.
     * **Carro 2 (Vermelho):** Centralizado na **Vaga 3** em $(X = 9.0, Y = -3.5)$.
     * **Carro 3 (Preto/Cinza):** Centralizado na **Vaga 4** em $(X = 12.0, Y = -3.5)$.
   * Deixe as **Vagas 2 $(X = 6.0)$ e 5 $(X = 15.0)$ livres**.

---

### 3.4. Parte 3 – Nó de Navegação Autônoma (`parking_inspector_bot/patrol_navigator_node.py`)
Implemente um nó ROS2 com máquina de estados finitos (FSM) calibrado para as novas coordenadas:
1. **Assinaturas e Publicações:**
   * Assina `/odom` (`nav_msgs/Odometry`) e `/ultrasonic/front` (`sensor_msgs/Range`).
   * Publica `/cmd_vel` (`geometry_msgs/Twist`), `/inspection_trigger` (`std_msgs/Int32`) e `/patrol_state` (`std_msgs/String`).
2. **Waypoints e Máquina de Estados:**
   * Pontos de inspeção alinhados ao centro de cada vaga de $3.0\text{ m}$ de largura:
     `waypoints_x = [3.0, 6.0, 9.0, 12.0, 15.0]` com `Y = 0.0` e orientação $\theta = 0.0\text{ rad}$.
   * Estados:
     * `START_DELAY`: Aguarda $2.0\text{ s}$ na estação inicial $(0.0, 0.0)$.
     * `MOVING_TO_SPOT`: Controlador Proporcional (P) para velocidade linear $v_x$ (até $0.50\text{ m/s}$) e controle angular $\omega_z$ (usando conversão quaternion $\rightarrow$ Euler yaw da odometria) para manter o robô alinhado no eixo $Y = 0.0$.
     * `INSPECTING_SPOT`: Ao chegar no ponto de inspeção (erro $< 0.08\text{ m}$), para o robô (`cmd_vel = 0`), publica o ID da vaga ($1$ a $5$) em `/inspection_trigger` e aguarda $2.5\text{ s}$ para coleta estável das amostras do ultrassom.
     * `RETURNING_TO_BASE`: Após inspecionar a Vaga 5 ($X = 15.0$), retorna até a estação inicial $(0.0, 0.0)$ e realinha para $\theta = 0.0\text{ rad}$.
     * `WAIT_NEXT_CYCLE`: Aguarda `patrol_interval_sec` (padrão $15.0\text{ s}$) na estação inicial e reinicia a ronda periódica automaticamente.
3. **Anticolisão:** Se `/ultrasonic/front` detectar obstáculo $< 0.40\text{ m}$ durante o deslocamento, para imediatamente e registra aviso no log.

---

### 3.5. Parte 4 – Nó de Detecção de Vagas por Ultrassom (`parking_inspector_bot/ultrasonic_detector_node.py`)
Implemente a classificação por **distância média** (Opção C) calibrada para vagas de $5.0\text{ m}$ de comprimento:
1. Assina `/ultrasonic/right` (`sensor_msgs/Range`) e `/inspection_trigger` (`std_msgs/Int32`).
2. Mantém um buffer circular (`collections.deque(maxlen=15)`) das leituras válidas recentes. Valores `inf` ou maiores que `max_range` são tratados como $7.0\text{ m}$.
3. Quando recebe o ID da vaga $N$ em `/inspection_trigger`:
   * Aguarda $1.5\text{ s}$ para garantir que o buffer contenha amostras colhidas com o robô parado em frente à vaga.
   * Calcula a **distância média ($\mu$)** e o **desvio padrão ($\sigma$)**.
   * Compara com o limiar configurável `occupancy_threshold_m = 4.2` metros (já que carro ocupado lê $\approx 1.22\text{ m}$ e vaga livre de $5.0\text{ m}$ lê $\approx 6.02\text{ m}$):
     * Se $\mu < 4.2\text{ m}$ $\rightarrow$ `"OCUPADA"`.
     * Se $\mu \ge 4.2\text{ m}$ $\rightarrow$ `"LIVRE"`.
   * Publica JSON em `/spot_status_update` (`std_msgs/String`):
     `{"spot_id": N, "status": "OCUPADA" | "LIVRE", "mean_distance_m": round(mu, 3), "std_dev_m": round(sigma, 3), "samples": len(buffer), "timestamp": ...}`.

---

### 3.6. Sistema Central de Monitoramento (`parking_inspector_bot/central_monitor_node.py`)
1. Assina `/spot_status_update`, `/odom` e `/patrol_state`.
2. **Painel em Tempo Real no Terminal:** Exibe tabela formatada com o status das 5 vagas, distância média medida, contagem de vagas livres/ocupadas e pose atual do robô. Salva também em `/tmp/parking_central_status.json`.
3. **Marcadores Visuais 3D no RViz2 (`/parking_markers`):**
   * Publica `visualization_msgs/MarkerArray` com cubos dimensionados exatamente como as vagas simuladas (**$2.8\text{ m}$ de largura $\times$ $4.8\text{ m}$ de comprimento $\times$ $0.08\text{ m}$ de altura**) posicionados em $(X_i, -3.5, 0.04)$ para $X_i \in [3.0, 6.0, 9.0, 12.0, 15.0]$, além de textos flutuantes `TEXT_VIEW_FACING` acima de cada vaga ($Z = 1.8\text{ m}$).
   * Cores: **Cinza** (não inspecionada), **Verde** (`LIVRE`), **Vermelho** (`OCUPADA`).

---

### 3.7. Launch File, RViz2 e Documentação (`launch/`, `rviz/`, `README.md`)
1. **`launch/sim_complete.launch.py`:** Inicia Gazebo com `parking_lot.world`, `robot_state_publisher` com o Xacro, faz o `spawn_entity` do robô em $(0.0, 0.0, 0.05)$, abre o `rviz2` com `parking_monitor.rviz` e inicia os nós `patrol_navigator`, `ultrasonic_detector` e `central_monitor`.
2. **`rviz/parking_monitor.rviz`:** Fixed Frame `odom`, exibindo `RobotModel`, `TF`, `Range` (`/ultrasonic/right` e `/ultrasonic/front`) e `MarkerArray` (`/parking_markers`).
3. **`README.md`:** Guia completo de compilação (`colcon build --symlink-install`), execução da simulação e explicação da arquitetura de nós e tópicos.

---

## 4. DIRETRIZES DE EXECUÇÃO IMEDIATA
1. Crie todos os diretórios e todos os arquivos listados acima agora mesmo no workspace.
2. Verifique cuidadosamente o fechamento de todas as tags XML no `.urdf.xacro` e no `.world`.
3. Garanta que todos os scripts Python tenham permissão de execução e cabeçalho `#!/usr/bin/env python3`.