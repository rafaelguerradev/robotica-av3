/*
 * ============================================================================
 * ROBO MOVEL AUTONOMO PARA MONITORAMENTO DE VAGAS DE ESTACIONAMENTO
 * Microcontrolador : ESP32-WROOM-32 (Arduino IDE / PlatformIO)
 * Sensores         : 2x Ultrassonicos HC-SR04 (Lateral Direito + Frontal)
 * Driver Motores   : Ponte H Dupla L298N (ou TB6612FNG)
 * Comunicacao      : Serial UART / USB 115200 baud (JSON + Comandos Dinamicos)
 * Compatibilidade  : ESP32 Arduino Core 2.x e 3.x
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

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

// =========================================================================
// MAPEAMENTO SEGURO DE PINOS DO ESP32-WROOM-32
// (Evitando strapping pins GPIO 0, 2, 12, 15 e input-only 34, 35, 36, 39)
// =========================================================================
const int PIN_ENA        = 25;  // PWM Motor Esquerdo (L298N ENA)
const int PIN_IN1        = 26;  // Direcao Motor Esquerdo IN1
const int PIN_IN2        = 27;  // Direcao Motor Esquerdo IN2

const int PIN_ENB        = 14;  // PWM Motor Direito (L298N ENB)
const int PIN_IN3        = 32;  // Direcao Motor Direito IN3
const int PIN_IN4        = 33;  // Direcao Motor Direito IN4

const int TRIG_RIGHT     = 18;  // Trigger HC-SR04 Lateral Direito (Vagas)
const int ECHO_RIGHT     = 19;  // Echo HC-SR04 Lateral Direito (Usar divisor 5V -> 3.3V!)

const int TRIG_FRONT     = 21;  // Trigger HC-SR04 Frontal (Anticolisao)
const int ECHO_FRONT     = 22;  // Echo HC-SR04 Frontal (Usar divisor 5V -> 3.3V!)

const int BTN_START      = 4;   // Botao fisico de inicio de missao (INPUT_PULLUP)
const int LED_STATUS     = 13;  // LED indicador de estado operacional

// =========================================================================
// CONSTANTES DE CONTROLE, PWM E SEGURANCA
// =========================================================================
const int PWM_FREQ_HZ             = 5000;
const int PWM_RESOLUTION_BITS     = 8;
const int PWM_CHANNEL_LEFT        = 0;
const int PWM_CHANNEL_RIGHT       = 1;

const unsigned long PULSE_TIMEOUT_US          = 25000UL; // 25 ms (~4.3 m)
const unsigned long AMOSTRAGEM_INTERVALO_MS   = 30UL;    // 30 ms entre pulsos
const unsigned long ESTABILIZACAO_PARADA_MS   = 400UL;   // 400 ms antes de medir vaga
const unsigned long TELEMETRIA_INTERVALO_MS   = 100UL;   // 10 Hz (100 ms)
const unsigned long FRONT_CHECK_INTERVALO_MS  = 60UL;    // Leitura frontal periodica
const unsigned long CMD_VEL_WATCHDOG_MS       = 1500UL;  // Timeout de seguranca Modo B
const unsigned long DEBOUNCE_BOTAO_MS         = 60UL;

const float MIN_DIST_VALIDA_M      = 0.02f;
const float MAX_DIST_VALIDA_M      = 3.50f;
const float DIST_PARADA_FRONTAL_M  = 0.25f;
const float SEPARACAO_RODAS_M      = 0.18f;
const float VEL_MAX_TELEOP_MS      = 0.45f;
const int   MAX_VAGAS_SUPORTADAS   = 20;

// =========================================================================
// MAQUINA DE ESTADOS E ESTRUTURAS DE DADOS
// =========================================================================
enum ModoOperacao {
  MODO_IDLE = 0,
  MODO_AUTONOMO = 1,
  MODO_TELEOP_ROS = 2
};

enum EstadoAutonomo {
  AUTO_IDLE = 0,
  AUTO_AVANCANDO_VAGA,
  AUTO_PAUSA_OBSTACULO_AVANCO,
  AUTO_ESTABILIZANDO_VAGA,
  AUTO_MEDINDO_VAGA,
  AUTO_GIRANDO_180_RETORNO,
  AUTO_RETORNANDO_BASE,
  AUTO_PAUSA_OBSTACULO_RETORNO,
  AUTO_GIRANDO_180_BASE
};

struct ResultadoVaga {
  int   spotId;
  float mediaDistM;
  float desvioPadraoM;
  float limiarM;
  bool  ocupada;
  int   amostrasValidas;
};

ModoOperacao   modoAtual          = MODO_IDLE;
EstadoAutonomo estadoAutonomo     = AUTO_IDLE;
ResultadoVaga  historicoVagas[MAX_VAGAS_SUPORTADAS];

int   vagaAlvoAtual               = 1;      // Indice 1..TOTAL_VAGAS
float odomXMetros                 = 0.0f;   // Posicao longitudinal estimada no corredor (m)
float odomYawRad                  = 0.0f;   // Orientacao estimada (0 = ida, pi = volta)
float alvoXMetros                 = 0.0f;   // Coordenada X alvo do deslocamento atual
float ultimaDistDireitaM          = 3.50f;
float ultimaDistFrontalM          = 3.50f;
float ultimoDesvioVagaM           = 0.0f;
int   ultimasAmostrasValidas      = 0;

float cmdVelVx                    = 0.0f;
float cmdVelWz                    = 0.0f;
bool  paradaEmergenciaSolicitada  = false;

unsigned long tsUltimaIntegracaoMs  = 0UL;
unsigned long tsInicioEstadoMs      = 0UL;
unsigned long tsUltimaTelemetriaMs  = 0UL;
unsigned long tsUltimaLeituraFront  = 0UL;
unsigned long tsUltimoCmdVelMs      = 0UL;
unsigned long tsUltimoLedMs         = 0UL;
unsigned long tsDebounceBotaoMs     = 0UL;

bool ultimoEstadoBrutoBotao = HIGH;
bool estadoEstavelBotao     = HIGH;
bool estadoLed              = false;

char bufferSerial[256];
size_t indiceSerial = 0;

// =========================================================================
// PROTOTIPOS DE FUNCOES
// =========================================================================
void configurarPWMHardware();
void escreverPWM(int pino, int canal, int dutyCycle);
void aplicarMotoresBruto(int pwmEsquerdoComSinal, int pwmDireitoComSinal);
void pararMotores();
void moverFrenteCruzeiro();
void girar180GrausEsquerda();
void aplicarControleCmdVel(float vx, float wz);

float calcularLimiarOcupacao();
float calcularPosicaoVagaX(int indiceVaga);
float lerDistanciaMetros(int trigPin, int echoPin);
float medirMediaVaga(int amostras = 15);

void iniciarCicloAutonomo();
void acionarParadaEmergencia(const char* motivo);
void publicarEventoVagaInspecionada(int spotId, float mediaM, float desvioM, int amostras, float limiarM, bool ocupada);
void publicarRelatorioFinalCiclo();
void publicarTelemetriaPeriodica();
void publicarConfiguracaoAtual();

bool extrairFloatJson(const char* jsonStr, const char* chave, float* valorSaida);
bool extrairStringJson(const char* jsonStr, const char* chave, char* destino, size_t tamDestino);
void processarLinhaComandoSerial(char* linha);
void processarComandosSerial();
void verificarBotaoFisico();
void atualizarIndicadorLed();
void atualizarOdometriaLinear(float velocidadeAssinadaMs);
void executarMaquinaEstadosAutonoma();

// =========================================================================
// CAMADA DE ABSTRACAO PWM (COMPATIVEL COM ARDUINO CORE ESP32 2.x E 3.x)
// =========================================================================
void configurarPWMHardware() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_ENA, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttach(PIN_ENB, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN_ENA, PWM_CHANNEL_LEFT);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN_ENB, PWM_CHANNEL_RIGHT);
#endif
}

void escreverPWM(int pino, int canal, int dutyCycle) {
  int dutyLimitado = constrain(dutyCycle, 0, 255);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  (void)canal;
  ledcWrite(pino, dutyLimitado);
#else
  (void)pino;
  ledcWrite(canal, dutyLimitado);
#endif
}

// =========================================================================
// CONTROLE DA PONTE H L298N / TB6612FNG
// =========================================================================
void aplicarMotoresBruto(int pwmEsquerdoComSinal, int pwmDireitoComSinal) {
  // Motor Esquerdo
  if (pwmEsquerdoComSinal > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    escreverPWM(PIN_ENA, PWM_CHANNEL_LEFT, pwmEsquerdoComSinal);
  } else if (pwmEsquerdoComSinal < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    escreverPWM(PIN_ENA, PWM_CHANNEL_LEFT, -pwmEsquerdoComSinal);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    escreverPWM(PIN_ENA, PWM_CHANNEL_LEFT, 0);
  }

  // Motor Direito
  if (pwmDireitoComSinal > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    escreverPWM(PIN_ENB, PWM_CHANNEL_RIGHT, pwmDireitoComSinal);
  } else if (pwmDireitoComSinal < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    escreverPWM(PIN_ENB, PWM_CHANNEL_RIGHT, -pwmDireitoComSinal);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    escreverPWM(PIN_ENB, PWM_CHANNEL_RIGHT, 0);
  }
}

void pararMotores() {
  aplicarMotoresBruto(0, 0);
}

void moverFrenteCruzeiro() {
  aplicarMotoresBruto(PWM_MOTOR_ESQ, PWM_MOTOR_DIR);
}

void girar180GrausEsquerda() {
  aplicarMotoresBruto(-PWM_MOTOR_ESQ, PWM_MOTOR_DIR);
}

void aplicarControleCmdVel(float vx, float wz) {
  float vEsq = vx - (wz * SEPARACAO_RODAS_M * 0.5f);
  float vDir = vx + (wz * SEPARACAO_RODAS_M * 0.5f);

  int pwmEsq = 0;
  int pwmDir = 0;

  if (fabsf(vEsq) > 0.01f) {
    float razaoEsq = constrain(fabsf(vEsq) / VEL_MAX_TELEOP_MS, 0.0f, 1.0f);
    int magEsq = static_cast<int>(90.0f + razaoEsq * (static_cast<float>(PWM_MOTOR_ESQ) - 90.0f));
    magEsq = constrain(magEsq, 0, 255);
    pwmEsq = (vEsq >= 0.0f) ? magEsq : -magEsq;
  }

  if (fabsf(vDir) > 0.01f) {
    float razaoDir = constrain(fabsf(vDir) / VEL_MAX_TELEOP_MS, 0.0f, 1.0f);
    int magDir = static_cast<int>(90.0f + razaoDir * (static_cast<float>(PWM_MOTOR_DIR) - 90.0f));
    magDir = constrain(magDir, 0, 255);
    pwmDir = (vDir >= 0.0f) ? magDir : -magDir;
  }

  aplicarMotoresBruto(pwmEsq, pwmDir);
}

// =========================================================================
// FUNCOES DE PARAMETRIZACAO DINAMICA E LEITURA ULTRASSONICA (OPCAO C)
// =========================================================================
float calcularLimiarOcupacao() {
  float limiarOcupacao = FOLGA_CORREDOR_M + (VAGA_COMPRIMENTO_M * 0.85f);
  return limiarOcupacao;
}

float calcularPosicaoVagaX(int indiceVaga) {
  return static_cast<float>(indiceVaga) * VAGA_LARGURA_M;
}

float lerDistanciaMetros(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duracaoUs = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (duracaoUs == 0UL) {
    // Sem eco dentro de 25 ms (~4.3 m): caminho livre alem do alcance util
    return MAX_DIST_VALIDA_M + 0.50f;
  }

  // Velocidade do som ~343 m/s (0.000343 m/us), ida e volta (/ 2)
  float distanciaMetros = (static_cast<float>(duracaoUs) * 0.000343f) * 0.5f;
  return distanciaMetros;
}

float medirMediaVaga(int amostras) {
  if (amostras <= 0) {
    amostras = 15;
  }

  float leiturasAceitas[40];
  int maxBuffer = (amostras > 40) ? 40 : amostras;
  int contagemValidas = 0;
  int contagemCampoAberto = 0;
  float somaDistancias = 0.0f;

  for (int i = 0; i < maxBuffer; i++) {
    processarComandosSerial();
    if (paradaEmergenciaSolicitada) {
      ultimasAmostrasValidas = 0;
      ultimoDesvioVagaM = 0.0f;
      return MAX_DIST_VALIDA_M;
    }

    float leituraM = lerDistanciaMetros(TRIG_RIGHT, ECHO_RIGHT);
    if (leituraM >= MIN_DIST_VALIDA_M && leituraM <= MAX_DIST_VALIDA_M) {
      leiturasAceitas[contagemValidas] = leituraM;
      somaDistancias += leituraM;
      contagemValidas++;
    } else if (leituraM > MAX_DIST_VALIDA_M) {
      contagemCampoAberto++;
    }

    // Aguarda 30 ms entre pulsos de forma cooperativa (millis) evitando ecos fantasmas
    if (i + 1 < maxBuffer) {
      unsigned long inicioEsperaMs = millis();
      while ((millis() - inicioEsperaMs) < AMOSTRAGEM_INTERVALO_MS) {
        processarComandosSerial();
        if (paradaEmergenciaSolicitada) {
          ultimasAmostrasValidas = contagemValidas;
          ultimoDesvioVagaM = 0.0f;
          return (contagemValidas > 0) ? (somaDistancias / static_cast<float>(contagemValidas)) : MAX_DIST_VALIDA_M;
        }
      }
    }
  }

  ultimasAmostrasValidas = contagemValidas;

  if (contagemValidas == 0) {
    ultimoDesvioVagaM = 0.0f;
    // Caso a pista fisica nao tenha parede de fundo e todos os pulsos excedam 3.50 m,
    // retorna o teto de campo aberto (3.50 m) para classificar corretamente como LIVRE.
    if (contagemCampoAberto > 0) {
      ultimaDistDireitaM = MAX_DIST_VALIDA_M;
      return MAX_DIST_VALIDA_M;
    }
    return MAX_DIST_VALIDA_M;
  }

  float mediaExata = somaDistancias / static_cast<float>(contagemValidas);
  float somaQuadrados = 0.0f;
  for (int j = 0; j < contagemValidas; j++) {
    float diff = leiturasAceitas[j] - mediaExata;
    somaQuadrados += (diff * diff);
  }
  ultimoDesvioVagaM = sqrtf(somaQuadrados / static_cast<float>(contagemValidas));
  ultimaDistDireitaM = mediaExata;
  return mediaExata;
}

// =========================================================================
// PUBLICACAO SERIAL JSON E RELATORIOS
// =========================================================================
const char* obterNomeEstadoAtual() {
  if (modoAtual == MODO_TELEOP_ROS) {
    return "TELEOP_CMD_VEL";
  }
  if (modoAtual == MODO_IDLE) {
    return "IDLE_BASE";
  }
  switch (estadoAutonomo) {
    case AUTO_AVANCANDO_VAGA:          return "MOVING_TO_SPOT";
    case AUTO_PAUSA_OBSTACULO_AVANCO:  return "OBSTACLE_STOP_FORWARD";
    case AUTO_ESTABILIZANDO_VAGA:      return "STABILIZING_AT_SPOT";
    case AUTO_MEDINDO_VAGA:            return "INSPECTING_SPOT";
    case AUTO_GIRANDO_180_RETORNO:     return "TURNING_180_RETURN";
    case AUTO_RETORNANDO_BASE:         return "RETURNING_TO_BASE";
    case AUTO_PAUSA_OBSTACULO_RETORNO: return "OBSTACLE_STOP_RETURN";
    case AUTO_GIRANDO_180_BASE:        return "ALIGNING_AT_BASE";
    default:                           return "IDLE_BASE";
  }
}

void publicarEventoVagaInspecionada(
    int spotId,
    float mediaM,
    float desvioM,
    int amostras,
    float limiarM,
    bool ocupada) {
  const char* statusStr = ocupada ? "OCUPADA" : "LIVRE";
  Serial.printf(
      "{\"event\":\"SPOT_INSPECTED\",\"spot_id\":%d,\"mean_dist_m\":%.2f,"
      "\"threshold_m\":%.2f,\"status\":\"%s\",\"vaga_largura_m\":%.2f,"
      "\"vaga_comprimento_m\":%.2f,\"std_dev_m\":%.3f,\"samples\":%d}\n",
      spotId,
      mediaM,
      limiarM,
      statusStr,
      VAGA_LARGURA_M,
      VAGA_COMPRIMENTO_M,
      desvioM,
      amostras);
}

void publicarRelatorioFinalCiclo() {
  int totalInspecionadas = constrain(TOTAL_VAGAS, 1, MAX_VAGAS_SUPORTADAS);
  int vagasOcupadas = 0;
  int vagasLivres = 0;

  for (int i = 0; i < totalInspecionadas; i++) {
    if (historicoVagas[i].ocupada) {
      vagasOcupadas++;
    } else {
      vagasLivres++;
    }
  }

  Serial.printf(
      "{\"event\":\"CYCLE_COMPLETED\",\"total_spots\":%d,\"free_spots\":%d,"
      "\"occupied_spots\":%d,\"odom_x\":%.2f,\"vaga_largura_m\":%.2f,"
      "\"vaga_comprimento_m\":%.2f}\n",
      totalInspecionadas,
      vagasLivres,
      vagasOcupadas,
      odomXMetros,
      VAGA_LARGURA_M,
      VAGA_COMPRIMENTO_M);

  Serial.println(F("================================================================"));
  Serial.println(F("   RELATORIO CONSOLIDADO DA RONDA AUTONOMA (ESP32-WROOM-32)"));
  Serial.println(F("================================================================"));
  Serial.printf(
      " Parametros: Largura=%.2f m | Comprimento=%.2f m | Limiar=%.2f m\n",
      VAGA_LARGURA_M,
      VAGA_COMPRIMENTO_M,
      calcularLimiarOcupacao());
  Serial.println(F("----------------------------------------------------------------"));
  for (int i = 0; i < totalInspecionadas; i++) {
    Serial.printf(
        " Vaga %d (X=%.2f m) -> %-8s | Media=%.2f m | Limiar=%.2f m | Amostras=%d\n",
        historicoVagas[i].spotId,
        calcularPosicaoVagaX(historicoVagas[i].spotId),
        historicoVagas[i].ocupada ? "OCUPADA" : "LIVRE",
        historicoVagas[i].mediaDistM,
        historicoVagas[i].limiarM,
        historicoVagas[i].amostrasValidas);
  }
  Serial.println(F("----------------------------------------------------------------"));
  Serial.printf(
      " RESUMO FINAL: LIVRES = %d | OCUPADAS = %d | Estacao Inicial X = %.2f m\n",
      vagasLivres,
      vagasOcupadas,
      odomXMetros);
  Serial.println(F("================================================================"));
}

void publicarTelemetriaPeriodica() {
  float distDirExibicao = (ultimaDistDireitaM > MAX_DIST_VALIDA_M) ? MAX_DIST_VALIDA_M : ultimaDistDireitaM;
  float distFrontExibicao = (ultimaDistFrontalM > MAX_DIST_VALIDA_M) ? MAX_DIST_VALIDA_M : ultimaDistFrontalM;

  Serial.printf(
      "{\"event\":\"TELEMETRY\",\"state\":\"%s\",\"odom_x\":%.3f,\"odom_yaw\":%.3f,"
      "\"dist_right\":%.3f,\"dist_front\":%.3f,\"vx\":%.3f,\"wz\":%.3f,"
      "\"vaga_largura_m\":%.2f,\"vaga_comprimento_m\":%.2f,\"threshold_m\":%.2f}\n",
      obterNomeEstadoAtual(),
      odomXMetros,
      odomYawRad,
      distDirExibicao,
      distFrontExibicao,
      cmdVelVx,
      cmdVelWz,
      VAGA_LARGURA_M,
      VAGA_COMPRIMENTO_M,
      calcularLimiarOcupacao());
}

void publicarConfiguracaoAtual() {
  Serial.printf(
      "{\"event\":\"CONFIG_UPDATED\",\"vaga_largura_m\":%.2f,\"vaga_comprimento_m\":%.2f,"
      "\"folga_corredor_m\":%.2f,\"threshold_m\":%.2f,\"total_vagas\":%d,"
      "\"velocidade_ms\":%.3f,\"pwm_esq\":%d,\"pwm_dir\":%d,\"tempo_giro_180_ms\":%lu}\n",
      VAGA_LARGURA_M,
      VAGA_COMPRIMENTO_M,
      FOLGA_CORREDOR_M,
      calcularLimiarOcupacao(),
      TOTAL_VAGAS,
      VELOCIDADE_CRUZEIRO_MS,
      PWM_MOTOR_ESQ,
      PWM_MOTOR_DIR,
      TEMPO_GIRO_180_MS);
}

// =========================================================================
// PARSER SERIAL LEVE (COMANDOS TEXTO + JSON SEM BIBLIOTECAS EXTERNAS)
// =========================================================================
bool extrairFloatJson(const char* jsonStr, const char* chave, float* valorSaida) {
  char padraoBusca[48];
  snprintf(padraoBusca, sizeof(padraoBusca), "\"%s\"", chave);
  const char* posChave = strstr(jsonStr, padraoBusca);
  if (posChave == nullptr) {
    return false;
  }
  const char* posDoisPontos = strchr(posChave + strlen(padraoBusca), ':');
  if (posDoisPontos == nullptr) {
    return false;
  }
  const char* cursor = posDoisPontos + 1;
  while (*cursor == ' ' || *cursor == '\t') {
    cursor++;
  }
  char* fimNumero = nullptr;
  float convertido = strtof(cursor, &fimNumero);
  if (fimNumero == cursor) {
    return false;
  }
  *valorSaida = convertido;
  return true;
}

bool extrairStringJson(const char* jsonStr, const char* chave, char* destino, size_t tamDestino) {
  char padraoBusca[48];
  snprintf(padraoBusca, sizeof(padraoBusca), "\"%s\"", chave);
  const char* posChave = strstr(jsonStr, padraoBusca);
  if (posChave == nullptr) {
    return false;
  }
  const char* posDoisPontos = strchr(posChave + strlen(padraoBusca), ':');
  if (posDoisPontos == nullptr) {
    return false;
  }
  const char* aspasInicio = strchr(posDoisPontos + 1, '\"');
  if (aspasInicio == nullptr) {
    return false;
  }
  aspasInicio++;
  const char* aspasFim = strchr(aspasInicio, '\"');
  if (aspasFim == nullptr) {
    return false;
  }
  size_t comprimento = static_cast<size_t>(aspasFim - aspasInicio);
  if (comprimento >= tamDestino) {
    comprimento = tamDestino - 1;
  }
  memcpy(destino, aspasInicio, comprimento);
  destino[comprimento] = '\0';
  return true;
}

void iniciarCicloAutonomo() {
  pararMotores();
  paradaEmergenciaSolicitada = false;
  modoAtual = MODO_AUTONOMO;
  vagaAlvoAtual = 1;
  odomXMetros = 0.0f;
  odomYawRad = 0.0f;
  cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
  cmdVelWz = 0.0f;
  alvoXMetros = calcularPosicaoVagaX(vagaAlvoAtual);
  estadoAutonomo = AUTO_AVANCANDO_VAGA;
  tsUltimaIntegracaoMs = millis();
  tsInicioEstadoMs = millis();

  Serial.printf(
      "{\"event\":\"MISSION_STARTED\",\"total_spots\":%d,\"first_target_x_m\":%.2f,"
      "\"vaga_largura_m\":%.2f,\"vaga_comprimento_m\":%.2f,\"threshold_m\":%.2f}\n",
      TOTAL_VAGAS,
      alvoXMetros,
      VAGA_LARGURA_M,
      VAGA_COMPRIMENTO_M,
      calcularLimiarOcupacao());

  moverFrenteCruzeiro();
}

void acionarParadaEmergencia(const char* motivo) {
  pararMotores();
  paradaEmergenciaSolicitada = true;
  modoAtual = MODO_IDLE;
  estadoAutonomo = AUTO_IDLE;
  cmdVelVx = 0.0f;
  cmdVelWz = 0.0f;
  Serial.printf(
      "{\"event\":\"EMERGENCY_STOP\",\"reason\":\"%s\",\"odom_x\":%.3f}\n",
      motivo,
      odomXMetros);
}

void processarLinhaComandoSerial(char* linha) {
  // Remove espacos iniciais e finais
  while (*linha == ' ' || *linha == '\t') {
    linha++;
  }
  size_t len = strlen(linha);
  while (len > 0 && (linha[len - 1] == ' ' || linha[len - 1] == '\t')) {
    linha[len - 1] = '\0';
    len--;
  }
  if (len == 0) {
    return;
  }

  // 1. Tratamento de payloads JSON (Modo B CMD_VEL ou configuracao dinamica)
  if (linha[0] == '{') {
    char cmdJson[32];
    if (extrairStringJson(linha, "cmd", cmdJson, sizeof(cmdJson))) {
      if (strcasecmp(cmdJson, "START") == 0) {
        iniciarCicloAutonomo();
        return;
      }
      if (strcasecmp(cmdJson, "STOP") == 0) {
        acionarParadaEmergencia("SERIAL_JSON_STOP");
        return;
      }
    }

    bool configAlterada = false;
    float valTemp = 0.0f;
    if (extrairFloatJson(linha, "vaga_largura_m", &valTemp) && valTemp > 0.10f) {
      VAGA_LARGURA_M = valTemp;
      configAlterada = true;
    }
    if (extrairFloatJson(linha, "vaga_comprimento_m", &valTemp) && valTemp > 0.10f) {
      VAGA_COMPRIMENTO_M = valTemp;
      configAlterada = true;
    }
    if (extrairFloatJson(linha, "folga_corredor_m", &valTemp) && valTemp >= 0.0f) {
      FOLGA_CORREDOR_M = valTemp;
      configAlterada = true;
    }
    if (configAlterada) {
      publicarConfiguracaoAtual();
    }

    float vxRecebido = 0.0f;
    float wzRecebido = 0.0f;
    bool temVx = extrairFloatJson(linha, "vx", &vxRecebido);
    bool temWz = extrairFloatJson(linha, "wz", &wzRecebido);
    if (temVx || temWz) {
      paradaEmergenciaSolicitada = false;
      modoAtual = MODO_TELEOP_ROS;
      estadoAutonomo = AUTO_IDLE;
      cmdVelVx = vxRecebido;
      cmdVelWz = wzRecebido;
      tsUltimoCmdVelMs = millis();
      tsUltimaIntegracaoMs = millis();
      aplicarControleCmdVel(cmdVelVx, cmdVelWz);
    }
    return;
  }

  // 2. Comandos textuais diretos via Monitor Serial / Ponte ROS2
  if (strcasecmp(linha, "START") == 0) {
    iniciarCicloAutonomo();
    return;
  }

  if (strcasecmp(linha, "STOP") == 0) {
    acionarParadaEmergencia("SERIAL_STOP_CMD");
    return;
  }

  if (strcasecmp(linha, "STATUS") == 0 || strcasecmp(linha, "CONFIG") == 0) {
    publicarConfiguracaoAtual();
    return;
  }

  if (strncasecmp(linha, "SET_LARGURA:", 12) == 0) {
    float novaLargura = atof(linha + 12);
    if (novaLargura >= 0.10f && novaLargura <= 10.0f) {
      VAGA_LARGURA_M = novaLargura;
      if (modoAtual == MODO_AUTONOMO && estadoAutonomo == AUTO_AVANCANDO_VAGA) {
        alvoXMetros = calcularPosicaoVagaX(vagaAlvoAtual);
      }
      publicarConfiguracaoAtual();
    }
    return;
  }

  if (strncasecmp(linha, "SET_COMPRIMENTO:", 16) == 0) {
    float novoComp = atof(linha + 16);
    if (novoComp >= 0.10f && novoComp <= 10.0f) {
      VAGA_COMPRIMENTO_M = novoComp;
      publicarConfiguracaoAtual();
    }
    return;
  }

  if (strncasecmp(linha, "SET_FOLGA:", 10) == 0) {
    float novaFolga = atof(linha + 10);
    if (novaFolga >= 0.0f && novaFolga <= 3.0f) {
      FOLGA_CORREDOR_M = novaFolga;
      publicarConfiguracaoAtual();
    }
    return;
  }

  if (strncasecmp(linha, "SET_VAGAS:", 10) == 0) {
    int novasVagas = atoi(linha + 10);
    if (novasVagas >= 1 && novasVagas <= MAX_VAGAS_SUPORTADAS) {
      TOTAL_VAGAS = novasVagas;
      publicarConfiguracaoAtual();
    }
    return;
  }

  if (strncasecmp(linha, "SET_VELOCIDADE:", 15) == 0) {
    float novaVel = atof(linha + 15);
    if (novaVel >= 0.05f && novaVel <= 2.0f) {
      VELOCIDADE_CRUZEIRO_MS = novaVel;
      publicarConfiguracaoAtual();
    }
    return;
  }

  if (strncasecmp(linha, "SET_PWM:", 8) == 0) {
    const char* virgula = strchr(linha + 8, ',');
    if (virgula != nullptr) {
      int novoEsq = atoi(linha + 8);
      int novoDir = atoi(virgula + 1);
      PWM_MOTOR_ESQ = constrain(novoEsq, 0, 255);
      PWM_MOTOR_DIR = constrain(novoDir, 0, 255);
      publicarConfiguracaoAtual();
    }
    return;
  }

  if (strncasecmp(linha, "SET_GIRO_MS:", 12) == 0) {
    long novoTempoGiro = atol(linha + 12);
    if (novoTempoGiro >= 100L && novoTempoGiro <= 10000L) {
      TEMPO_GIRO_180_MS = static_cast<unsigned long>(novoTempoGiro);
      publicarConfiguracaoAtual();
    }
    return;
  }
}

void processarComandosSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (indiceSerial > 0) {
        bufferSerial[indiceSerial] = '\0';
        processarLinhaComandoSerial(bufferSerial);
        indiceSerial = 0;
      }
    } else if (indiceSerial + 1 < sizeof(bufferSerial)) {
      bufferSerial[indiceSerial++] = c;
    } else {
      indiceSerial = 0;
    }
  }
}

// =========================================================================
// LEITURA DE BOTAO COM DEBOUNCE, LED DE STATUS E ODOMETRIA
// =========================================================================
void verificarBotaoFisico() {
  bool leituraAtual = (digitalRead(BTN_START) == HIGH);
  unsigned long agoraMs = millis();

  if (leituraAtual != ultimoEstadoBrutoBotao) {
    tsDebounceBotaoMs = agoraMs;
    ultimoEstadoBrutoBotao = leituraAtual;
  }

  if ((agoraMs - tsDebounceBotaoMs) >= DEBOUNCE_BOTAO_MS) {
    if (leituraAtual != estadoEstavelBotao) {
      estadoEstavelBotao = leituraAtual;
      // Botao pressionado (INPUT_PULLUP -> nivel logico LOW)
      if (!estadoEstavelBotao) {
        if (modoAtual == MODO_AUTONOMO) {
          acionarParadaEmergencia("BUTTON_TOGGLE_STOP");
        } else {
          iniciarCicloAutonomo();
        }
      }
    }
  }
}

void atualizarIndicadorLed() {
  unsigned long agoraMs = millis();
  if (modoAtual == MODO_IDLE) {
    // Piscada lenta (1 Hz) em espera
    if ((agoraMs - tsUltimoLedMs) >= 500UL) {
      tsUltimoLedMs = agoraMs;
      estadoLed = !estadoLed;
      digitalWrite(LED_STATUS, estadoLed ? HIGH : LOW);
    }
  } else if (modoAtual == MODO_AUTONOMO) {
    if (estadoAutonomo == AUTO_MEDINDO_VAGA || estadoAutonomo == AUTO_ESTABILIZANDO_VAGA) {
      digitalWrite(LED_STATUS, HIGH);
    } else if ((agoraMs - tsUltimoLedMs) >= 150UL) {
      tsUltimoLedMs = agoraMs;
      estadoLed = !estadoLed;
      digitalWrite(LED_STATUS, estadoLed ? HIGH : LOW);
    }
  } else {
    // Modo B Teleop: piscada dupla/rapida (2.5 Hz)
    if ((agoraMs - tsUltimoLedMs) >= 200UL) {
      tsUltimoLedMs = agoraMs;
      estadoLed = !estadoLed;
      digitalWrite(LED_STATUS, estadoLed ? HIGH : LOW);
    }
  }
}

void atualizarOdometriaLinear(float velocidadeAssinadaMs) {
  unsigned long agoraMs = millis();
  unsigned long deltaMs = agoraMs - tsUltimaIntegracaoMs;
  tsUltimaIntegracaoMs = agoraMs;
  if (deltaMs > 250UL) {
    deltaMs = 0UL;
  }
  float dtSeg = static_cast<float>(deltaMs) * 0.001f;
  odomXMetros += velocidadeAssinadaMs * dtSeg;
  if (odomXMetros < 0.0f) {
    odomXMetros = 0.0f;
  }
}

// =========================================================================
// EXECUCAO DA MAQUINA DE ESTADOS DO MODO A (AUTONOMO EMBARCADO)
// =========================================================================
void executarMaquinaEstadosAutonoma() {
  unsigned long agoraMs = millis();

  // Monitoramento continuo do ultrassonico frontal (a cada 60 ms) durante deslocamentos
  bool emDeslocamentoLinear = (
      estadoAutonomo == AUTO_AVANCANDO_VAGA ||
      estadoAutonomo == AUTO_PAUSA_OBSTACULO_AVANCO ||
      estadoAutonomo == AUTO_RETORNANDO_BASE ||
      estadoAutonomo == AUTO_PAUSA_OBSTACULO_RETORNO);

  if (emDeslocamentoLinear && (agoraMs - tsUltimaLeituraFront) >= FRONT_CHECK_INTERVALO_MS) {
    tsUltimaLeituraFront = agoraMs;
    ultimaDistFrontalM = lerDistanciaMetros(TRIG_FRONT, ECHO_FRONT);
  }

  switch (estadoAutonomo) {
    case AUTO_IDLE: {
      pararMotores();
      break;
    }

    case AUTO_AVANCANDO_VAGA: {
      if (ultimaDistFrontalM < DIST_PARADA_FRONTAL_M) {
        pararMotores();
        cmdVelVx = 0.0f;
        estadoAutonomo = AUTO_PAUSA_OBSTACULO_AVANCO;
        Serial.printf(
            "{\"event\":\"OBSTACLE_ALERT\",\"dist_front_m\":%.2f,\"odom_x\":%.2f}\n",
            ultimaDistFrontalM,
            odomXMetros);
        break;
      }

      moverFrenteCruzeiro();
      cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
      atualizarOdometriaLinear(VELOCIDADE_CRUZEIRO_MS);

      alvoXMetros = calcularPosicaoVagaX(vagaAlvoAtual);
      if (odomXMetros >= alvoXMetros) {
        odomXMetros = alvoXMetros;
        pararMotores();
        cmdVelVx = 0.0f;
        tsInicioEstadoMs = agoraMs;
        estadoAutonomo = AUTO_ESTABILIZANDO_VAGA;
      }
      break;
    }

    case AUTO_PAUSA_OBSTACULO_AVANCO: {
      pararMotores();
      tsUltimaIntegracaoMs = agoraMs;
      if (ultimaDistFrontalM >= (DIST_PARADA_FRONTAL_M + 0.05f)) {
        moverFrenteCruzeiro();
        cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
        estadoAutonomo = AUTO_AVANCANDO_VAGA;
      }
      break;
    }

    case AUTO_ESTABILIZANDO_VAGA: {
      pararMotores();
      if ((agoraMs - tsInicioEstadoMs) >= ESTABILIZACAO_PARADA_MS) {
        estadoAutonomo = AUTO_MEDINDO_VAGA;
      }
      break;
    }

    case AUTO_MEDINDO_VAGA: {
      pararMotores();
      float mediaVagaM = medirMediaVaga(15);
      if (paradaEmergenciaSolicitada) {
        break;
      }

      float limiarOcupacao = calcularLimiarOcupacao();
      bool vagaOcupada = (mediaVagaM < limiarOcupacao);

      int idxRegistro = constrain(vagaAlvoAtual - 1, 0, MAX_VAGAS_SUPORTADAS - 1);
      historicoVagas[idxRegistro].spotId = vagaAlvoAtual;
      historicoVagas[idxRegistro].mediaDistM = mediaVagaM;
      historicoVagas[idxRegistro].desvioPadraoM = ultimoDesvioVagaM;
      historicoVagas[idxRegistro].limiarM = limiarOcupacao;
      historicoVagas[idxRegistro].ocupada = vagaOcupada;
      historicoVagas[idxRegistro].amostrasValidas = ultimasAmostrasValidas;

      publicarEventoVagaInspecionada(
          vagaAlvoAtual,
          mediaVagaM,
          ultimoDesvioVagaM,
          ultimasAmostrasValidas,
          limiarOcupacao,
          vagaOcupada);

      if (vagaAlvoAtual < TOTAL_VAGAS) {
        vagaAlvoAtual++;
        alvoXMetros = calcularPosicaoVagaX(vagaAlvoAtual);
        tsUltimaIntegracaoMs = millis();
        moverFrenteCruzeiro();
        cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
        estadoAutonomo = AUTO_AVANCANDO_VAGA;
      } else {
        // Ultima vaga inspecionada: inicia giro de 180 graus para retornar a X = 0.0 m
        tsInicioEstadoMs = millis();
        girar180GrausEsquerda();
        cmdVelVx = 0.0f;
        estadoAutonomo = AUTO_GIRANDO_180_RETORNO;
      }
      break;
    }

    case AUTO_GIRANDO_180_RETORNO: {
      girar180GrausEsquerda();
      if ((agoraMs - tsInicioEstadoMs) >= TEMPO_GIRO_180_MS) {
        pararMotores();
        odomYawRad = 3.1415926f;
        // Garante que a distancia total de retorno parte de TOTAL_VAGAS * VAGA_LARGURA_M
        odomXMetros = calcularPosicaoVagaX(TOTAL_VAGAS);
        tsUltimaIntegracaoMs = millis();
        moverFrenteCruzeiro();
        cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
        estadoAutonomo = AUTO_RETORNANDO_BASE;
      }
      break;
    }

    case AUTO_RETORNANDO_BASE: {
      if (ultimaDistFrontalM < DIST_PARADA_FRONTAL_M) {
        pararMotores();
        cmdVelVx = 0.0f;
        estadoAutonomo = AUTO_PAUSA_OBSTACULO_RETORNO;
        Serial.printf(
            "{\"event\":\"OBSTACLE_ALERT\",\"dist_front_m\":%.2f,\"odom_x\":%.2f}\n",
            ultimaDistFrontalM,
            odomXMetros);
        break;
      }

      moverFrenteCruzeiro();
      cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
      atualizarOdometriaLinear(-VELOCIDADE_CRUZEIRO_MS);

      if (odomXMetros <= 0.0f) {
        odomXMetros = 0.0f;
        pararMotores();
        cmdVelVx = 0.0f;
        tsInicioEstadoMs = agoraMs;
        girar180GrausEsquerda();
        estadoAutonomo = AUTO_GIRANDO_180_BASE;
      }
      break;
    }

    case AUTO_PAUSA_OBSTACULO_RETORNO: {
      pararMotores();
      tsUltimaIntegracaoMs = agoraMs;
      if (ultimaDistFrontalM >= (DIST_PARADA_FRONTAL_M + 0.05f)) {
        moverFrenteCruzeiro();
        cmdVelVx = VELOCIDADE_CRUZEIRO_MS;
        estadoAutonomo = AUTO_RETORNANDO_BASE;
      }
      break;
    }

    case AUTO_GIRANDO_180_BASE: {
      girar180GrausEsquerda();
      if ((agoraMs - tsInicioEstadoMs) >= TEMPO_GIRO_180_MS) {
        pararMotores();
        odomXMetros = 0.0f;
        odomYawRad = 0.0f;
        cmdVelVx = 0.0f;
        cmdVelWz = 0.0f;
        estadoAutonomo = AUTO_IDLE;
        modoAtual = MODO_IDLE;
        publicarRelatorioFinalCiclo();
      }
      break;
    }
  }
}

// =========================================================================
// SETUP E LOOP PRINCIPAL (NAO-BLOQUEANTE BASEADO EM MILLIS)
// =========================================================================
void setup() {
  Serial.begin(115200);

  // Configura pinos de direcao da Ponte H L298N
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  configurarPWMHardware();
  pararMotores();

  // Configura sensores ultrassonicos HC-SR04
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  digitalWrite(TRIG_RIGHT, LOW);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  digitalWrite(TRIG_FRONT, LOW);

  // Configura botao fisico e LED de status
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  tsUltimaIntegracaoMs = millis();
  tsUltimaTelemetriaMs = millis();

  Serial.println();
  Serial.println(F("{\"event\":\"BOOT_READY\",\"device\":\"ESP32-WROOM-32\",\"firmware\":\"1.0.0\"}"));
  publicarConfiguracaoAtual();
}

void loop() {
  unsigned long agoraMs = millis();

  // 1. Leitura continua de comandos Serial (START, STOP, SET_LARGURA, SET_COMPRIMENTO, JSON CMD_VEL)
  processarComandosSerial();

  // 2. Monitoramento nao-bloqueante do botao fisico (GPIO 4) e LED (GPIO 13)
  verificarBotaoFisico();
  atualizarIndicadorLed();

  // 3. Execucao do Modo A (Autonomo Embarcado) ou Modo B (Teleop CMD_VEL ROS2)
  if (modoAtual == MODO_AUTONOMO) {
    executarMaquinaEstadosAutonoma();
  } else if (modoAtual == MODO_TELEOP_ROS) {
    // Watchdog de seguranca caso o fluxo /cmd_vel seja interrompido
    if ((agoraMs - tsUltimoCmdVelMs) > CMD_VEL_WATCHDOG_MS) {
      pararMotores();
      cmdVelVx = 0.0f;
      cmdVelWz = 0.0f;
      modoAtual = MODO_IDLE;
    } else {
      unsigned long deltaMs = agoraMs - tsUltimaIntegracaoMs;
      tsUltimaIntegracaoMs = agoraMs;
      if (deltaMs <= 250UL) {
        float dt = static_cast<float>(deltaMs) * 0.001f;
        odomYawRad += cmdVelWz * dt;
        odomXMetros += (cmdVelVx * cosf(odomYawRad)) * dt;
      }
    }
  } else {
    tsUltimaIntegracaoMs = agoraMs;
  }

  // 4. Telemetria periodica a 10 Hz (100 ms) para a Ponte ROS2 e Monitor Serial
  if ((agoraMs - tsUltimaTelemetriaMs) >= TELEMETRIA_INTERVALO_MS) {
    tsUltimaTelemetriaMs = agoraMs;

    // Em Modo Teleop ou Idle, atualiza os dois sensores alternadamente para telemetria ao vivo
    if (modoAtual == MODO_TELEOP_ROS || modoAtual == MODO_IDLE) {
      ultimaDistFrontalM = lerDistanciaMetros(TRIG_FRONT, ECHO_FRONT);
      ultimaDistDireitaM = lerDistanciaMetros(TRIG_RIGHT, ECHO_RIGHT);
    }

    publicarTelemetriaPeriodica();
  }
}
