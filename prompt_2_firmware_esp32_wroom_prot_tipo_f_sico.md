# PROMPT DE ENGENHARIA (PARTE 2): FIRMWARE COMPLETO PARA ESP32-WROOM E INTEGRAÇÃO DO PROTÓTIPO FÍSICO

Atue como um **Engenheiro Sênior de Sistemas Embarcados e Robótica Autônoma (ESP32 + ROS2)**.
Sua tarefa é criar no meu workspace do VS Code, do zero e **sem usar nenhum código incompleto, resumo ou placeholder (`TODO`, `pass`, `...`)**, todos os arquivos necessários para o **protótipo físico baseado no microcontrolador ESP32-WROOM-32**, utilizando **2 sensores ultrassônicos HC-SR04** e driver Ponte H **L298N** (ou TB6612FNG).

---

## 1. REQUISITOS DIMENSIONAIS E DE PARAMETRIZAÇÃO (OBRIGATÓRIO)

No código embarcado do **ESP32-WROOM**, as dimensões das vagas físicas **NÃO podem ser números fixos espalhados pelo código**. Elas devem ser definidas no topo do arquivo através de variáveis/parâmetros configuráveis de fácil alteração (tanto no código quanto em tempo de execução via monitor serial):

1. **Variável de Largura da Vaga (`VAGA_LARGURA_M`):**
   * Define o espaçamento longitudinal ao longo do corredor entre o centro de uma vaga e a próxima.
   * **Valor inicial obrigatório:** **`1.60`** metros (`float VAGA_LARGURA_M = 1.60f;`).
   * *Uso no código:* O robô deve utilizar `VAGA_LARGURA_M` para calcular automaticamente a distância a percorrer até cada um dos 5 pontos de inspeção ($X_1 = 1 \times \text{VAGA\_LARGURA\_M}$, $X_2 = 2 \times \text{VAGA\_LARGURA\_M}$, ..., $X_5 = 5 \times \text{VAGA\_LARGURA\_M}$) e a distância total de retorno até a estação inicial ($0.0\text{ m}$).

2. **Variável de Comprimento / Profundidade da Vaga (`VAGA_COMPRIMENTO_M`):**
   * Define a profundidade lateral da vaga medida a partir da linha de entrada até o fundo da vaga.
   * **Valor inicial obrigatório:** **`0.68`** metros (`float VAGA_COMPRIMENTO_M = 0.68f;`).
   * *Uso no código:* Junto com a variável de folga do corredor (`float FOLGA_CORREDOR_M = 0.15f;`), o código deve calcular dinamicamente o limiar de detecção de ocupação:
     `float limiarOcupacao = FOLGA_CORREDOR_M + (VAGA_COMPRIMENTO_M * 0.85f);`
     Se a distância média medida pelo sensor ultrassônico lateral for menor que `limiarOcupacao`, a vaga é classificada como **OCUPADA**; caso contrário, é classificada como **LIVRE**.

3. **Configuração Dinâmica via Serial (Bônus para Apresentação):**
   * Permita também ajustar essas variáveis sem precisar recompilar o ESP32, aceitando comandos JSON ou texto pela Serial (`115200` baud), por exemplo:
     * `SET_LARGURA:1.60`
     * `SET_COMPRIMENTO:0.68`
     * `START` (inicia ciclo autônomo de inspeção)
     * `STOP` (parada de emergência)

---

## 2. ESTRUTURA DE ARQUIVOS A SER CRIADA

Crie a pasta `firmware_prototipo/` (e o nó de ponte ROS2 caso o pacote `parking_inspector_bot` já exista no workspace):

~~~text
firmware_prototipo/
├── esp32_ultrasonic_bot/
│   └── esp32_ultrasonic_bot.ino
├── platformio.ini
└── pinout_e_montagem_esp32_wroom.md
~~~

*(Nota: Caso a pasta `parking_inspector_bot/` já exista no workspace, crie também o nó `parking_inspector_bot/hardware_bridge_node.py` e o launch `launch/physical_prototype.launch.py` para conectar o ESP32-WROOM ao RViz2 e ao Sistema Central de Monitoramento).*

---

## 3. ESPECIFICAÇÃO TÉCNICA DETALHADA

### 3.1. Firmware Principal (`firmware_prototipo/esp32_ultrasonic_bot/esp32_ultrasonic_bot.ino`)
Escreva um firmware C++ completo, limpo, não-bloqueante (`millis()`) e compilável tanto na **Arduino IDE** (placa *ESP32 Dev Module*) quanto no **PlatformIO**:

1. **Mapeamento Seguro de Pinos do ESP32-WROOM-32:**
   * Evite pinos de strapping problemáticos (`GPIO 0`, `GPIO 2`, `GPIO 12`, `GPIO 15`) e lembre-se que `GPIO 34, 35, 36, 39` são apenas entrada (*Input Only*).
   * Use a seguinte pinagem segura:
     * **Motor Esquerdo (L298N):** `ENA = GPIO 25` (PWM via `ledcAttach` / `ledcSetup`), `IN1 = GPIO 26`, `IN2 = GPIO 27`.
     * **Motor Direito (L298N):** `ENB = GPIO 14` (PWM), `IN3 = GPIO 32`, `IN4 = GPIO 33`.
     * **Sensor Ultrassônico Lateral Direito (HC-SR04):** `TRIG_RIGHT = GPIO 18`, `ECHO_RIGHT = GPIO 19`.
     * **Sensor Ultrassônico Frontal (HC-SR04):** `TRIG_FRONT = GPIO 21`, `ECHO_FRONT = GPIO 22`.
     * **Botão Físico de Início de Missão:** `BTN_START = GPIO 4` (`INPUT_PULLUP`).
     * **LED de Status:** `LED_STATUS = GPIO 13`.

2. **Variáveis de Parametrização e Calibração no Topo do Código:**
   ~~~cpp
   // =========================================================================
   // PARÂMETROS CONFIGURÁVEIS DAS VAGAS (ALTERE CONFORME A PISTA FÍSICA)
   // =========================================================================
   float VAGA_LARGURA_M     = 1.60f;  // Largura de cada vaga em metros (distância entre centros)
   float VAGA_COMPRIMENTO_M = 0.68f;  // Comprimento/profundidade da vaga em metros
   float FOLGA_CORREDOR_M   = 0.15f;  // Distância lateral do sensor até a linha de entrada da vaga
   int   TOTAL_VAGAS        = 5;      // Quantidade total de vagas a inspecionar

   // Calibração de locomoção (odometria por tempo/velocidade ou encoders)
   float VELOCIDADE_CRUZEIRO_MS = 0.32f; // Velocidade linear calibrada do robô (m/s)
   int   PWM_MOTOR_ESQ          = 185;   // PWM (0-255) calibrado para andar em linha reta
   int   PWM_MOTOR_DIR          = 185;   // PWM (0-255) calibrado para andar em linha reta
   unsigned long TEMPO_GIRO_180_MS = 1150; // Tempo calibrado para girar 180 graus no retorno
   ~~~

3. **Leitura Ultrassônica com Filtro Estatístico (Opção C):**
   * Implemente `float lerDistanciaMetros(int trigPin, int echoPin)` usando `pulseIn(echoPin, HIGH, 25000)` (timeout de $25\text{ ms} \approx 4.3\text{ m}$).
   * Implemente `float medirMediaVaga(int amostras = 15)` que coleta 15 leituras consecutivas com intervalo de $30\text{ ms}$ entre pulsos (para evitar ecos fantasmas), descarta leituras fora da faixa válida ($0.02\text{ m}$ a $3.50\text{ m}$) e retorna a **distância média** exata em metros.

4. **Dois Modos de Operação Integrados:**
   * **Modo A – Autônomo Embarcado (Stand-alone na Pista Física):**
     * Acionado ao pressionar o botão físico (`GPIO 4`) ou receber `"START"` via Serial.
     * Para cada vaga $i = 1 \dots \text{TOTAL\_VAGAS}$:
       1. Avança pelo corredor percorrendo a distância exata definida por `VAGA_LARGURA_M` (monitorando continuamente o ultrassônico frontal para parada de emergência se houver obstáculo a $< 0.25\text{ m}$).
       2. Para os motores completamente e aguarda $400\text{ ms}$ para estabilização mecânica.
       3. Executa `medirMediaVaga(15)` no sensor lateral direito.
       4. Compara o resultado com `limiarOcupacao = FOLGA_CORREDOR_M + (VAGA_COMPRIMENTO_M * 0.85f)`.
       5. Envia pela Serial (`115200` baud) o JSON estruturado para o ROS2 / Sistema Central:
          `{"event":"SPOT_INSPECTED","spot_id":1,"mean_dist_m":0.34,"threshold_m":0.73,"status":"OCUPADA","vaga_largura_m":1.60,"vaga_comprimento_m":0.68}`
     * Após a última vaga, executa manobra de retorno à estação inicial ($X = 0.0\text{ m}$), para os motores e imprime o relatório consolidado das 5 vagas.
   * **Modo B – Teleoperação / Controle Direto pelo ROS2 (`CMD_VEL`):**
     * Se receber comandos JSON `{"vx": 0.25, "wz": 0.0}` pela Serial, aciona os motores proporcionalmente via PWM e publica telemetria periódica a $10\text{ Hz}$ com `dist_right`, `dist_front` e `odom_x`.

---

### 3.2. Configuração PlatformIO (`firmware_prototipo/platformio.ini`)
Crie um arquivo `platformio.ini` pronto para compilação com `env:esp32dev`, `platform = espressif32`, `board = esp32dev`, `framework = arduino` e `monitor_speed = 115200`.

---

### 3.3. Guia de Montagem, Pinout e Calibração (`firmware_prototipo/pinout_e_montagem_esp32_wroom.md`)
Crie um manual técnico completo em Português contendo:
1. **Lista de Materiais (BOM):** ESP32-WROOM-32, Ponte H L298N, 2x sensores HC-SR04, chassi 2WD com motores DC, suporte de baterias 18650 (7.4V) e resistores.
2. **Tabela Completa de Conexões (Pinout):** Mapeamento pino a pino entre ESP32-WROOM, L298N e os dois HC-SR04.
3. **Alerta Crítico de Hardware (Divisor de Tensão 5V $\rightarrow$ 3.3V):** Diagrama em ASCII mostrando como ligar um resistor de $1\text{ k}\Omega$ e um resistor de $2\text{ k}\Omega$ nos pinos `ECHO` dos sensores HC-SR04 (que operam em $5\text{V}$) antes de conectá-los aos GPIOs do ESP32-WROOM (que suporta no máximo $3.3\text{V}$), evitando queimar o microcontrolador.
4. **Procedimento Passo a Passo de Calibração para a Apresentação:**
   * Como ajustar `PWM_MOTOR_ESQ` e `PWM_MOTOR_DIR` para compensar diferenças entre os dois motores DC e andar em linha reta.
   * Como calibrar `VELOCIDADE_CRUZEIRO_MS` para que o robô pare exatamente no centro de cada vaga de `VAGA_LARGURA_M = 1.60` metros.
   * Como validar o limiar com `VAGA_COMPRIMENTO_M = 0.68` metros usando um obstáculo/carrinho na bancada.

---

### 3.4. Ponte ROS2 para o Protótipo Físico (`parking_inspector_bot/hardware_bridge_node.py` e `launch/physical_prototype.launch.py`)
*(Crie estes arquivos integrados ao pacote `parking_inspector_bot` se ele existir no workspace, ou crie a estrutura mínima do pacote para executá-los):*
1. **`hardware_bridge_node.py`:**
   * Abre comunicação serial com o ESP32-WROOM (`pyserial`, `/dev/ttyUSB0`, `115200` baud).
   * Converte as leituras do ESP32 em tópicos ROS2 (`/ultrasonic/right`, `/ultrasonic/front`, `/odom`, `/spot_status_update`) e envia `/cmd_vel` para o ESP32, permitindo que o **Sistema Central de Monitoramento** e o **RViz2** exibam em tempo real o que o protótipo físico está detectando na pista!
2. **`launch/physical_prototype.launch.py`:**
   * Inicia `hardware_bridge_node`, `central_monitor_node` e `rviz2` configurados para a demonstração ao vivo com o robô real.

---

## 4. DIRETRIZES DE EXECUÇÃO IMEDIATA
1. Crie todos os arquivos agora mesmo no workspace, garantindo compatibilidade total com a API do **ESP32 Arduino Core** (suportando tanto Arduino Core 2.x quanto 3.x na configuração de PWM `ledc`).
2. Não deixe nenhuma função incompleta ou comentada.