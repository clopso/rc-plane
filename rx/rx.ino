/*
 * ============================================================================
 * RECEPTOR RC PARA AEROMODELO - ESP32 + NRF24L01 (COM FAILSAFE)
 * ============================================================================
 * Bibliotecas necessárias (instalar via Gerenciador de Bibliotecas):
 *  - RF24 (por TMRh20)
 *  - ESP32Servo (por Kevin Harrington)
 * ============================================================================
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>

// ============================================================================
// 1. DEFINIÇÃO DE PINOS
// ============================================================================
// Pinos do NRF24L01 (VSPI padrão: SCK=18, MISO=19, MOSI=23)
#define PIN_CE   4
#define PIN_CSN  5

// Pinos PWM dos atuadores
#define PIN_SERVO_1  26  // Servo 1 (Aileron ou Leme)
#define PIN_SERVO_2  25  // Servo 2 (Profundor)
#define PIN_ESC      27  // ESC (Motor)

// ============================================================================
// 2. CONFIGURAÇÕES DE RÁDIO E SEGURANÇA (FAILSAFE)
// ============================================================================
// Endereço idêntico ao configurado no Transmissor (TX)
const byte PIPE_ADDRESS[6] = "AERO1";

// Tempo máximo sem receber pacotes antes de cortar o motor (500 ms = 0,5 s)
const unsigned long FAILSAFE_TIMEOUT_MS = 500;
const unsigned long DEBUG_INTERVAL_MS   = 150;

// Mude para 'true' se o seu avião usa Leme (CH4) no Servo 1 em vez de Aileron (CH1)
#define USAR_LEME_NO_SERVO_1  false

// ============================================================================
// 3. ESTRUTURA DO PACOTE RC (IDÊNTICA AO TRANSMISSOR)
// ============================================================================
struct __attribute__((packed)) PacoteRC {
  uint16_t aileron;   // Canal 1: 1000 a 2000 us
  uint16_t elevator;  // Canal 2: 1000 a 2000 us
  uint16_t throttle;  // Canal 3: 1000 a 2000 us
  uint16_t rudder;    // Canal 4: 1000 a 2000 us
  uint8_t  swLeft;    // Botão Esquerdo
  uint8_t  swRight;   // Botão Direito
};

// ============================================================================
// 4. OBJETOS E VARIÁVEIS GLOBAIS
// ============================================================================
RF24 radio(PIN_CE, PIN_CSN);
PacoteRC dadosRX;

Servo servo1;
Servo servo2;
Servo escMotor;

unsigned long ultimoPacoteMs = 0;
unsigned long ultimoDebugMs  = 0;
bool failsafeAtivo = true;

// ============================================================================
// 5. FUNÇÕES DE ACIONAMENTO E FAILSAFE
// ============================================================================
void aplicarFailsafe() {
  // Segurança máxima: zera o motor e centraliza as superfícies de comando
  escMotor.writeMicroseconds(1000);
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
}

void atualizarSaidasPWM() {
  // Garante que os pulsos estejam sempre dentro da faixa segura de 1000-2000 us
  uint16_t chMotor     = constrain(dadosRX.throttle, 1000, 2000);
  uint16_t chProfundor = constrain(dadosRX.elevator, 1000, 2000);

  #if USAR_LEME_NO_SERVO_1
    uint16_t chServo1 = constrain(dadosRX.rudder, 1000, 2000);
  #else
    uint16_t chServo1 = constrain(dadosRX.aileron, 1000, 2000);
  #endif

  escMotor.writeMicroseconds(chMotor);
  servo1.writeMicroseconds(chServo1);
  servo2.writeMicroseconds(chProfundor);
}

// ============================================================================
// 6. CONFIGURAÇÃO INICIAL (SETUP)
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== INICIALIZANDO RECEPTOR RC (ESP32 + NRF24L01) ===");

  // Aloca timers PWM de hardware do ESP32 para os servos e ESC
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  // Configura frequência padrão de 50 Hz para aeromodelismo
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  escMotor.setPeriodHertz(50);

  // Anexa os pinos definindo os limites mínimo e máximo (1000 a 2000 us)
  servo1.attach(PIN_SERVO_1, 1000, 2000);
  servo2.attach(PIN_SERVO_2, 1000, 2000);
  escMotor.attach(PIN_ESC, 1000, 2000);

  // Inicia em Failsafe (motor em 1000 us para armar o ESC com segurança)
  aplicarFailsafe();

  // Inicializa o módulo NRF24L01
  if (!radio.begin()) {
    Serial.println("[ERRO CRÍTICO] Módulo NRF24L01 não detectado no RX!");
    while (true) {
      aplicarFailsafe();
      delay(1000);
    }
  }

  // Configurações idênticas às do Transmissor
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setPayloadSize(sizeof(PacoteRC));

  radio.openReadingPipe(1, PIPE_ADDRESS);
  radio.startListening(); // Define o módulo como Receptor (RX)

  Serial.println("[OK] Receptor pronto. Aguardando sinal do transmissor...");
}

// ============================================================================
// 7. LOOP PRINCIPAL
// ============================================================================
void loop() {
  unsigned long agora = millis();

  // Verifica se chegou algum pacote novo do transmissor
  if (radio.available()) {
    radio.read(&dadosRX, sizeof(PacoteRC));
    ultimoPacoteMs = agora;
    failsafeAtivo  = false;

    atualizarSaidasPWM();
  }

  // Verifica perda de sinal (Failsafe)
  if ((agora - ultimoPacoteMs) > FAILSAFE_TIMEOUT_MS) {
    if (!failsafeAtivo) {
      failsafeAtivo = true;
      aplicarFailsafe();
    }
  }

  // Monitoramento no Serial
  if (agora - ultimoDebugMs >= DEBUG_INTERVAL_MS) {
    ultimoDebugMs = agora;

    if (failsafeAtivo) {
      Serial.println("[FAILSAFE ATIVO] Sem sinal do TX -> Motor: 1000 | Servos: 1500");
    } else {
      Serial.print("RX OK -> CH1(AIL): "); Serial.print(dadosRX.aileron);
      Serial.print(" | CH2(ELE): "); Serial.print(dadosRX.elevator);
      Serial.print(" | CH3(THR): "); Serial.print(dadosRX.throttle);
      Serial.print(" | CH4(RUD): "); Serial.print(dadosRX.rudder);
      Serial.print(" | SW: "); Serial.print(dadosRX.swLeft); Serial.print("/"); Serial.println(dadosRX.swRight);
    }
  }
}