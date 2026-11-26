# Manual Técnico de Montagem, Pinout e Calibração — Protótipo Físico ESP32-WROOM-32

Este documento descreve a lista de materiais, o diagrama completo de ligações elétricas, a proteção obrigatória de nível lógico ($5\text{V} \rightarrow 3.3\text{V}$) e o roteiro prático de calibração em pista do robô autônomo de monitoramento de estacionamento.

---

## 1. Lista de Materiais (BOM — Bill of Materials)

| Item | Qtd. | Componente | Especificação / Função no Protótipo |
| :--- | :---: | :--- | :--- |
| 1 | 1 | **Microcontrolador ESP32-WROOM-32** | Placa DevKit V1 (30 ou 38 pinos), 3.3V lógico, UART USB 115200 baud |
| 2 | 1 | **Driver Ponte H Dupla L298N** | Controle bidirecional e PWM dos 2 motores DC (alternativa: TB6612FNG) |
| 3 | 2 | **Sensor Ultrassônico HC-SR04** | 1x Lateral Direito (medição de vagas) + 1x Frontal (anticolisão) |
| 4 | 1 | **Kit Chassi Robô Móvel 2WD** | Base acrílica/MDF, 2 motores DC com redução (3V–6V), 2 rodas e 1 roda boba (*caster*) |
| 5 | 1 | **Suporte de Baterias 2S (18650)** | 2x células Li-Ion 18650 em série ($7.4\text{V}$ nominal / $8.4\text{V}$ carregadas) com chave Liga/Desliga |
| 6 | 2 | **Resistor de $1\text{ k}\Omega$ ($1/4\text{ W}$)** | Ramo superior ($R_1$) dos dois divisores de tensão para os pinos `ECHO` |
| 7 | 2 | **Resistor de $2\text{ k}\Omega$ ($1/4\text{ W}$)** | Ramo inferior ($R_2$) ao `GND` dos dois divisores de tensão (pode usar $2.2\text{ k}\Omega$) |
| 8 | 1 | **Botão Táctil (*Push-Button*)** | Acionamento físico de início/parada de missão no `GPIO 4` (`INPUT_PULLUP`) |
| 9 | 1 | **LED Difuso $5\text{ mm}$ + Resistor $220\text{ }\Omega$** | Sinalização visual de estado operacional conectado ao `GPIO 13` |
| 10 | 1 | **Mini Protoboard e Cabos Jumper** | Jumpers Macho-Fêmea e Macho-Macho para barramento de alimentação e divisores |

---

## 2. Tabela Completa de Conexões (Pinout Seguro ESP32-WROOM-32)

> **Nota de Engenharia sobre GPIOs do ESP32:** Foram evitados os pinos de *boot strapping* (`GPIO 0`, `GPIO 2`, `GPIO 12`, `GPIO 15`) para impedir falhas de inicialização/gravação e evitados os pinos somente entrada (`GPIO 34`, `GPIO 35`, `GPIO 36`, `GPIO 39`) em saídas de controle.

### 2.1. ESP32-WROOM-32 $\leftrightarrow$ Driver Ponte H L298N

> **Importante:** Remova os *jumpers* físicos dos pinos `ENA` e `ENB` na placa L298N para habilitar o controle de velocidade por PWM do ESP32.

| Pino ESP32-WROOM | Pino L298N | Função |
| :---: | :---: | :--- |
| `GPIO 25` | `ENA` | Velocidade PWM do Motor Esquerdo ($5\text{ kHz}$, 8 bits) |
| `GPIO 26` | `IN1` | Sentido de rotação A do Motor Esquerdo |
| `GPIO 27` | `IN2` | Sentido de rotação B do Motor Esquerdo |
| `GPIO 14` | `ENB` | Velocidade PWM do Motor Direito ($5\text{ kHz}$, 8 bits) |
| `GPIO 32` | `IN3` | Sentido de rotação A do Motor Direito |
| `GPIO 33` | `IN4` | Sentido de rotação B do Motor Direito |
| `GND` | `GND` | **Terra Comum Obrigatório** entre ESP32, L298N e Baterias |

### 2.2. ESP32-WROOM-32 $\leftrightarrow$ Sensores Ultrassônicos HC-SR04

| Sensor | Pino HC-SR04 | Conexão no Circuito / ESP32-WROOM | Observação Elétrica |
| :--- | :---: | :---: | :--- |
| **Lateral Direito (Vagas)** | `VCC` | Barramento `+5V` (Saída 5V L298N ou `VIN` USB) | Alimentação de $5\text{V}$ exigida pelo HC-SR04 |
| **Lateral Direito (Vagas)** | `TRIG` | `GPIO 18` (Ligação direta) | Pulso de saída $3.3\text{V}$ do ESP32 é aceito pelo HC-SR04 |
| **Lateral Direito (Vagas)** | `ECHO` | `GPIO 19` **VIA DIVISOR DE TENSÃO** | **NUNCA ligue direto!** Reduzir $5\text{V} \rightarrow 3.3\text{V}$ |
| **Lateral Direito (Vagas)** | `GND` | Barramento `GND` Comum | Referência comum |
| **Frontal (Anticolisão)** | `VCC` | Barramento `+5V` | Alimentação de $5\text{V}$ exigida pelo HC-SR04 |
| **Frontal (Anticolisão)** | `TRIG` | `GPIO 21` (Ligação direta) | Disparo ultrassônico de $10\text{ }\mu\text{s}$ |
| **Frontal (Anticolisão)** | `ECHO` | `GPIO 22` **VIA DIVISOR DE TENSÃO** | **NUNCA ligue direto!** Reduzir $5\text{V} \rightarrow 3.3\text{V}$ |
| **Frontal (Anticolisão)** | `GND` | Barramento `GND` Comum | Referência comum |

### 2.3. Interface Humana e Alimentação

| Componente | Terminal 1 | Terminal 2 | Observação |
| :--- | :---: | :---: | :--- |
| **Botão `START`** | `GPIO 4` | `GND` | Usa resistor interno `INPUT_PULLUP` (dispensa resistor externo) |
| **LED de Status** | `GPIO 13` (via $220\text{ }\Omega$) | `GND` | Pisca lento em espera, aceso fixo medindo vaga, rápido navegando |
| **Bateria 2S ($7.4\text{V}$)** | Polo Positivo (`+`) $\rightarrow$ Chave $\rightarrow$ `+12V` (VMS) L298N | Polo Negativo (`-`) $\rightarrow$ `GND` L298N | Se o *jumper* `5V_EN` da L298N estiver fechado, o pino `+5V` da L298N pode alimentar o pino `VIN` ($5\text{V}$) do ESP32 e os dois HC-SR04 |

---

## 3. ALERTA CRÍTICO DE HARDWARE: Divisor de Tensão ($5\text{V} \rightarrow 3.3\text{V}$) nos Pinos `ECHO`

> **[!CAUTION]**
> Os pinos GPIOs do **ESP32-WROOM-32 NÃO são tolerantes a $5\text{V}$** (tensão máxima absoluta de entrada = $3.6\text{V}$). O pino `ECHO` do sensor HC-SR04 emite nível lógico alto de **$5.0\text{V}$**. Conectar `ECHO` diretamente ao ESP32 pode danificar permanentemente o microcontrolador.

Monte **dois circuitos divisores resistivos idênticos** na protoboard — um para o `ECHO_RIGHT` (`GPIO 19`) e outro para o `ECHO_FRONT` (`GPIO 22`):

$$V_{\text{GPIO}} = V_{\text{ECHO}} \times \frac{R_2}{R_1 + R_2} = 5.0\text{ V} \times \frac{2\text{ k}\Omega}{1\text{ k}\Omega + 2\text{ k}\Omega} = 3.33\text{ V}$$

```text
   HC-SR04 (5V)                                      ESP32-WROOM-32 (3.3V)
  +------------+                                    +---------------------+
  |            |                                    |                     |
  |       TRIG |<-----------------------------------| GPIO 18 (ou GPIO 21)|
  |            |                                    |                     |
  |            |         R1 = 1 kOhm                |                     |
  |       ECHO |---------[  1 kOhm  ]-------+------>| GPIO 19 (ou GPIO 22)|
  |   (Saida   |                            |       |  (Entrada ~3.33 V)  |
  |    5.0 V)  |                            |       +---------------------+
  +------------+                            |
                                           ---
                                           | | R2 = 2 kOhm (ou 2.2 kOhm)
                                           | |
                                           ---
                                            |
                                            +------> GND COMUM (ESP32 + L298N + Bateria)
```

---

## 4. Procedimento Passo a Passo de Calibração para a Apresentação

O firmware permite calibrar o robô **em tempo real pelo Monitor Serial (`115200` baud)** sem precisar recompilar o código a cada teste, e depois gravar os valores definitivos no topo de `esp32_ultrasonic_bot.ino`.

### Passo 1 — Ajustar `PWM_MOTOR_ESQ` e `PWM_MOTOR_DIR` (Andar em Linha Reta)
Motores DC com caixa de redução apresentam pequenas diferenças mecânicas de rotação para o mesmo valor de PWM.
1. Posicione o robô alinhado sobre uma linha reta no piso (ou fita crepe marcando o eixo do corredor).
2. Conecte o cabo USB (ou abra o Monitor Serial a `115200` baud) e envie `START` com `SET_VAGAS:2` para um teste curto de avanço.
3. Observe a trajetória do chassi:
   * **Se o robô desviar para a DIREITA:** o motor esquerdo está mais rápido que o direito. Reduza o esquerdo ou aumente o direito, enviando na Serial:
     ```text
     SET_PWM:178,185
     ```
   * **Se o robô desviar para a ESQUERDA:** o motor direito está mais rápido que o esquerdo. Reduza o direito:
     ```text
     SET_PWM:185,178
     ```
4. Repita até o robô percorrer $3\text{ m}$ a $5\text{ m}$ mantendo-se centrado no corredor, e anote os dois valores finais em `PWM_MOTOR_ESQ` e `PWM_MOTOR_DIR`.

### Passo 2 — Calibrar `VELOCIDADE_CRUZEIRO_MS` para `VAGA_LARGURA_M = 1.60` m
O robô calcula o tempo de deslocamento até cada vaga dividindo a distância `VAGA_LARGURA_M` por `VELOCIDADE_CRUZEIRO_MS`.
1. Marque no chão a **Estação Inicial (`X = 0.00 m`)** e o centro da **Vaga 1 (`X = 1.60 m`)** usando uma trena.
2. Configure na Serial `SET_VAGAS:1` e `SET_LARGURA:1.60`, posicione o sensor lateral do robô sobre `X = 0.00 m` e envie `START`.
3. Meça com a trena a distância real $d_{\text{real}}$ percorrida até a primeira parada do robô e aplique a regra de três direta:
   $$\text{VELOCIDADE\_NOVA} = \text{VELOCIDADE\_ATUAL} \times \frac{d_{\text{real}}}{1.60}$$
   * *Exemplo prático:* Com `VELOCIDADE_CRUZEIRO_MS = 0.32`, se o robô parou em $d_{\text{real}} = 1.75\text{ m}$ (passou do ponto), calcule $0.32 \times (1.75 / 1.60) = 0.35\text{ m/s}$ e envie:
     ```text
     SET_VELOCIDADE:0.35
     ```
   * Se o robô parou antes do ponto em $d_{\text{real}} = 1.45\text{ m}$, calcule $0.32 \times (1.45 / 1.60) = 0.29\text{ m/s}$ e envie:
     ```text
     SET_VELOCIDADE:0.29
     ```
4. Ajuste também o giro de meia-volta (`TEMPO_GIRO_180_MS`): observe se ao final da última vaga o robô gira exatamente $180^\circ$. Caso gire menos de $180^\circ$, aumente o tempo via `SET_GIRO_MS:1250`; caso passe de $180^\circ$, reduza via `SET_GIRO_MS:1050`.
5. Volte para as 5 vagas enviando `SET_VAGAS:5`.

### Passo 3 — Validar `VAGA_COMPRIMENTO_M = 0.68` m na Bancada e na Pista
1. Com os valores padrões (`FOLGA_CORREDOR_M = 0.15` m e `VAGA_COMPRIMENTO_M = 0.68` m), o firmware calcula automaticamente:
   $$\text{limiarOcupacao} = 0.15 + (0.68 \times 0.85) = 0.728\text{ m} \approx 0.73\text{ m}$$
2. Abra o Monitor Serial (`115200` baud) — mesmo em modo `IDLE`, o ESP32 publica telemetria JSON a $10\text{ Hz}$ contendo `"dist_right"` e `"threshold_m":0.73`.
3. **Teste de Vaga Ocupada:** Coloque uma caixa ou carrinho de protótipo a uma distância entre $0.20\text{ m}$ e $0.65\text{ m}$ do sensor lateral direito (`HC-SR04`). Verifique que `"dist_right"` lê valor menor que `0.73` e que, ao inspecionar a vaga, o JSON reporta `"status":"OCUPADA"`.
4. **Teste de Vaga Livre:** Retire o obstáculo (deixando a vaga vazia até o fundo $> 0.80\text{ m}$ ou campo aberto). Verifique que `"dist_right"` supera `0.73` e que o JSON reporta `"status":"LIVRE"`.
5. Caso a largura da pista mude no dia da apresentação, basta enviar `SET_LARGURA:<novo_valor>` e `SET_COMPRIMENTO:<novo_valor>` pelo terminal ou atualizar as 4 linhas no topo de `esp32_ultrasonic_bot.ino`.
