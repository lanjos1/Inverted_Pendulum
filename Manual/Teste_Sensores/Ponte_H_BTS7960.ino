#include <Arduino.h>

// ===== CONFIGURAÇÃO DE PINOS (AJUSTE CONFORME SUA MONTAGEM) =====
const int RPWM = 23;   // Pino PWM para rotação no sentido HORÁRIO (Direita)
const int LPWM = 18;   // Pino PWM para rotação no sentido ANTI-HORÁRIO (Esquerda)
const int R_EN = 19;   // Enable do lado direito (fixo em HIGH)
const int L_EN = 4;    // Enable do lado esquerdo (substitui o GPIO21)

// ===== CONFIGURAÇÃO DO PWM =====
const uint32_t freqPWM = 20000;    // Frequência de 20kHz (ideal para BTS7960)
const uint8_t resolution = 8;      // Resolução de 8 bits (0-255)
const int pwmMax = 255;            // Valor máximo do PWM

// ===== VARIÁVEIS DE CONTROLE =====
int potencia = 0;                  // Valor atual do PWM (0-255)
const int incremento = 5;          // Passo de incremento do PWM
bool direcaoFrente = true;         // Sentido do motor (true = frente, false = trás)
unsigned long ultimaAtualizacao = 0;
unsigned long ultimaMudancaDirecao = 0;
const unsigned long intervaloAtualizacao = 50;   // Tempo entre incrementos (ms)
const unsigned long intervaloDirecao = 1000;     // Tempo para mudar de direção (ms)

// ===== FUNÇÃO PARA CONFIGURAR PINOS E PWM =====
void setup() {
  Serial.begin(115200);
  
  // Configura os pinos de enable (sempre HIGH para habilitar o BTS7960)
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  
  // Configuração dos canais PWM (RPWM e LPWM)
  ledcSetup(0, freqPWM, resolution);  // Canal 0 para RPWM
  ledcSetup(1, freqPWM, resolution);  // Canal 1 para LPWM
  ledcAttachPin(RPWM, 0);             // Associa RPWM ao canal 0
  ledcAttachPin(LPWM, 1);             // Associa LPWM ao canal 1
  
  Serial.println("Sistema iniciado: Controle de motor com BTS7960");
}

// ===== LOOP PRINCIPAL =====
void loop() {
  unsigned long agora = millis();
  
  // ---- CONTROLE DE VELOCIDADE (PWM GRADUAL) ----
  if (agora - ultimaAtualizacao >= intervaloAtualizacao) {
    ultimaAtualizacao = agora;
    
    // Incrementa a potência até o máximo
    if (potencia < pwmMax) {
      potencia += incremento;
      if (potencia > pwmMax) potencia = pwmMax;
    }
    
    // Aplica o PWM conforme a direção
    if (direcaoFrente) {
      ledcWrite(0, potencia);  // Ativa RPWM (sentido horário)
      ledcWrite(1, 0);         // Desativa LPWM
    } else {
      ledcWrite(0, 0);         // Desativa RPWM
      ledcWrite(1, potencia);  // Ativa LPWM (sentido anti-horário)
    }
    
    // Exibe informações no Serial Monitor
    Serial.print("Direção: ");
    Serial.print(direcaoFrente ? "Frente (RPWM)" : "Trás (LPWM)");
    Serial.print(" | Potência: ");
    Serial.print(map(potencia, 0, pwmMax, 0, 100));
    Serial.println("%");
  }
  
  // ---- CONTROLE DE MUDANÇA DE DIREÇÃO ----
  if (agora - ultimaMudancaDirecao >= intervaloDirecao) {
    ultimaMudancaDirecao = agora;
    direcaoFrente = !direcaoFrente;  // Inverte o sentido
    
    // Reinicia a potência para zero ao mudar de direção
    potencia = 0;
    Serial.println("--- Mudança de direção ---");
  }
}

/*
**Conexões**

| BTS7960 |  ESP32  |
| ------- | :-----: |
| VCC     |  3.3V   |
| GND     |   GND   |
| RPWM    | GPIO 23 |
| LPWM    | GPIO 18 |
| R_EN    | GPIO 19 |
| L_EN    | GPIO 4  |
*/
