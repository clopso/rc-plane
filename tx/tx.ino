#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ============================================================================
// 1. DEFINIÇÃO DE PINOS
// ============================================================================
#define PIN_CE   4
#define PIN_CSN  5

#define PIN_AILERON   32  // Canal 1: Joystick Direito - Eixo X
#define PIN_ELEVATOR  33  // Canal 2: Joystick Direito - Eixo Y
#define PIN_THROTTLE  35  // Canal 3: Joystick Esquerdo - Eixo Y (Motor)
#define PIN_RUDDER    34  // Canal 4: Joystick Esquerdo - Eixo X

#define PIN_SW_LEFT   25
#define PIN_SW_RIGHT  26

// ============================================================================
// 2. CALIBRAÇÃO E PARÂMETROS DE RÁDIO
// ============================================================================
#define INVERT_AILERON   false
#define INVERT_ELEVATOR  false
#define INVERT_THROTTLE  false
#define INVERT_RUDDER    false

const int DEADBAND_CENTER   = 20;  // Zona morta ao redor de 1500 us
const int DEADBAND_THROTTLE = 25;  // Zona morta no mínimo do motor (1000 us)
const float FILTER_ALPHA    = 0.25f;

const unsigned long TX_INTERVAL_MS    = 20;   // 50 Hz
const unsigned long DEBUG_INTERVAL_MS = 150;
const byte PIPE_ADDRESS[6] = "AERO1";

// ============================================================================
// 3. ESTRUTURA DO PACOTE RC
// ============================================================================
struct __attribute__((packed)) PacoteRC {
  uint16_t aileron;
  uint16_t elevator;
  uint16_t throttle;
  uint16_t rudder;
  uint8_t  swLeft;
  uint8_t  swRight;
};

RF24 radio(PIN_CE, PIN_CSN);
PacoteRC dadosTX;

// Centros reais capturados na inicialização (para eixos com mola)
int centroADC_Aileron  = 2048;
int centroADC_Elevator = 2048;
int centroADC_Rudder   = 2048;

// Estados do filtro exponencial
float filtAileron  = 2048.0f;
float filtElevator = 2048.0f;
float filtThrottle = 0.0f;
float filtRudder   = 2048.0f;

unsigned long ultimoEnvioMs = 0;
unsigned long ultimoDebugMs = 0;
bool ultimoStatusEnvio = false;

// ============================================================================
// 4. FUNÇÕES DE CALIBRAÇÃO, FILTRO E MAPEAMENTO
// ============================================================================
int calibrarCentroPino(uint8_t pino) {
  uint32_t soma = 0;
  for (int i = 0; i < 30; i++) {
    soma += analogRead(pino);
    delay(5);
  }
  int media = soma / 30;
  // Limita a faixa aceitável para evitar erro caso o stick seja segurado ao ligar
  return constrain(media, 1300, 2700);
}

float lerCanalFiltrado(uint8_t pino, float &estadoAnterior) {
  uint32_t soma = 0;
  for (uint8_t i = 0; i < 4; i++) {
    soma += analogRead(pino);
  }
  float leituraAtual = soma / 4.0f;
  estadoAnterior = (FILTER_ALPHA * leituraAtual) + ((1.0f - FILTER_ALPHA) * estadoAnterior);
  return estadoAnterior;
}

// Mapeamento assimétrico: garante 1500 exato no centro real e 1000/2000 nas pontas
uint16_t mapearSuperficie(float valorADC, int centroADC, bool inverter) {
  int raw = constrain((int)valorADC, 0, 4095);

  if (inverter) {
    raw = 4095 - raw;
    centroADC = 4095 - centroADC;
  }

  int pulso;
  if (raw < centroADC) {
    pulso = map(raw, 0, centroADC, 1000, 1500);
  } else {
    pulso = map(raw, centroADC, 4095, 1500, 2000);
  }

  pulso = constrain(pulso, 1000, 2000);

  if (abs(pulso - 1500) <= DEADBAND_CENTER) {
    pulso = 1500;
  }

  return (uint16_t)pulso;
}

// Mapeamento linear para o acelerador (0 -> 1000 us, máximo -> 2000 us)
uint16_t mapearThrottle(float valorADC, bool inverter) {
  int raw = constrain((int)valorADC, 0, 4095);
  if (inverter) {
    raw = 4095 - raw;
  }

  int pulso = map(raw, 0, 4095, 1000, 2000);
  pulso = constrain(pulso, 1000, 2000);

  if (pulso <= (1000 + DEADBAND_THROTTLE)) {
    pulso = 1000;
  }

  return (uint16_t)pulso;
}

// ============================================================================
// 5. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== CALIBRANDO CENTRO DOS STICKS (NAO TOQUE NOS JOYSTICKS) ===");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_SW_LEFT, INPUT_PULLUP);
  pinMode(PIN_SW_RIGHT, INPUT_PULLUP);

  // Mede o ponto de repouso real dos 3 comandos de superfície
  centroADC_Aileron  = calibrarCentroPino(PIN_AILERON);
  centroADC_Elevator = calibrarCentroPino(PIN_ELEVATOR);
  centroADC_Rudder   = calibrarCentroPino(PIN_RUDDER);

  filtAileron  = centroADC_Aileron;
  filtElevator = centroADC_Elevator;
  filtRudder   = centroADC_Rudder;
  filtThrottle = analogRead(PIN_THROTTLE);

  Serial.print("Centros ADC -> AIL: "); Serial.print(centroADC_Aileron);
  Serial.print(" | ELE: "); Serial.print(centroADC_Elevator);
  Serial.print(" | RUD: "); Serial.println(centroADC_Rudder);

  if (!radio.begin()) {
    Serial.println("[ERRO CRÍTICO] Módulo NRF24L01 não detectado!");
    while (true) {
      delay(1000);
    }
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(3, 8);
  radio.setPayloadSize(sizeof(PacoteRC));

  radio.openWritingPipe(PIPE_ADDRESS);
  radio.stopListening();

  Serial.println("[OK] Transmissor pronto a 50Hz.");
}

// ============================================================================
// 6. LOOP PRINCIPAL
// ============================================================================
void loop() {
  unsigned long agora = millis();

  if (agora - ultimoEnvioMs >= TX_INTERVAL_MS) {
    ultimoEnvioMs = agora;

    float rawAileron  = lerCanalFiltrado(PIN_AILERON,  filtAileron);
    float rawElevator = lerCanalFiltrado(PIN_ELEVATOR, filtElevator);
    float rawThrottle = lerCanalFiltrado(PIN_THROTTLE, filtThrottle);
    float rawRudder   = lerCanalFiltrado(PIN_RUDDER,   filtRudder);

    dadosTX.aileron  = mapearSuperficie(rawAileron,  centroADC_Aileron,  INVERT_AILERON);
    dadosTX.elevator = mapearSuperficie(rawElevator, centroADC_Elevator, INVERT_ELEVATOR);
    dadosTX.throttle = mapearThrottle(rawThrottle, INVERT_THROTTLE);
    dadosTX.rudder   = mapearSuperficie(rawRudder,   centroADC_Rudder,   INVERT_RUDDER);

    dadosTX.swLeft  = (digitalRead(PIN_SW_LEFT)  == LOW) ? 1 : 0;
    dadosTX.swRight = (digitalRead(PIN_SW_RIGHT) == LOW) ? 1 : 0;

    ultimoStatusEnvio = radio.write(&dadosTX, sizeof(PacoteRC));
  }

  if (agora - ultimoDebugMs >= DEBUG_INTERVAL_MS) {
    ultimoDebugMs = agora;

    Serial.print("CH1(AIL): "); Serial.print(dadosTX.aileron);
    Serial.print(" | CH2(ELE): "); Serial.print(dadosTX.elevator);
    Serial.print(" | CH3(THR): "); Serial.print(dadosTX.throttle);
    Serial.print(" | CH4(RUD): "); Serial.print(dadosTX.rudder);
    Serial.print(" | SW: "); Serial.print(dadosTX.swLeft); Serial.print("/"); Serial.print(dadosTX.swRight);
    Serial.print(" | Link: ");
    Serial.println(ultimoStatusEnvio ? "OK" : "FALHA (Sem ACK)");
  }
}