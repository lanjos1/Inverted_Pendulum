#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// VL53L0X
Adafruit_VL53L0X lox1;
Adafruit_VL53L0X lox2;
#define LOX1_XSHUT_PIN 26
#define LOX2_XSHUT_PIN 27

// Encoder rotacional (pêndulo)
#define ENCODER_A 34
#define ENCODER_B 35

// Motor do carrinho
#define RPWM 23
#define LPWM 18
#define R_EN 19
#define L_EN 4
#define freqPWM 20000
#define resolution 9
#define MAX_PWM 511

unsigned long timestamp = 0;

// Parâmetros do sistema (do MATLAB)
const float Ts = 0.05; // 50 ms (mesmo do seu código)
const float Kd[4] = {-6900.3 ,  -3282.9  ,  8299.6  ,  1281.2}; // Ganhos LQR discretos

// Matrizes do observador (calculadas no MATLAB)
const float Ld[4][2] = {
  { 0.9817, -0.0003 },
  { 19.6340, -0.0056 },
  { -0.0030, 0.9776 },
  { -0.0596, 19.5512 }
};

// Estado e estimativa
float x = 0, x_prev = 0;        // Posição medida
float x_hat[4] = {0};           // Estado estimado [x; x_dot; phi; phi_dot]
float y[2] = {0};               // Saída medida [x; phi]
float u = 0;                    // Sinal de controle

// Encoder rotacional
volatile int contador = 0;
volatile unsigned long ultimaInterrupcao = 0;
const unsigned long debounceTime = 200; // us

void IRAM_ATTR leEncoder() {
  unsigned long tempoAtual = micros();
  if (tempoAtual - ultimaInterrupcao < debounceTime) return;
  ultimaInterrupcao = tempoAtual;

  static int ultimoEstadoA = LOW;
  int estadoAtualA = digitalRead(ENCODER_A);

  if (estadoAtualA != ultimoEstadoA) {
    int estadoB = digitalRead(ENCODER_B);
    contador += (estadoB != estadoAtualA) ? 1 : -1;
    ultimoEstadoA = estadoAtualA;
  }
}

// Função para ler posição x com sensores VL53L0X
float ler_posicao_mm() {
  VL53L0X_RangingMeasurementData_t m1, m2;
  lox1.rangingTest(&m1, false);
  lox2.rangingTest(&m2, false);

  if (m1.RangeStatus != 4 && m2.RangeStatus != 4) {
    int d1 = m1.RangeMilliMeter;
    int d2 = m2.RangeMilliMeter;
    return (d1 - d2) / 2 - 20; // média em mm
  }
  return 0;
}

// Conversão encoder -> radianos
float ler_phi_rad() {
  float ganho_encoder = 0.005236;
  noInterrupts();
  int c = contador;
  interrupts();
  return c * ganho_encoder;
}

// Atualiza o observador de estado
void atualizar_observador(float u, float y[2]) {
  // Predição (modelo do sistema)
  float x_hat_pred[4] = {0};
  
  // Matriz Ad do sistema discreto (simplificada - deve ser ajustada conforme seu sistema)
  // Estas são aproximações - você deve usar as matrizes exatas do seu sistema
  x_hat_pred[0] = x_hat[0] + Ts * x_hat[1];
  x_hat_pred[1] = x_hat[1] + Ts * (0.1*u - 0.5*x_hat[1] - 0.1*x_hat[2]); // Ajuste esses parâmetros
  x_hat_pred[2] = x_hat[2] + Ts * x_hat[3];
  x_hat_pred[3] = x_hat[3] + Ts * (9.8*sin(x_hat[2]) + 0.1*u); // Ajuste esses parâmetros

  // Correção (feedback do erro de medição)
  float y_hat[2] = {x_hat_pred[0], x_hat_pred[2]}; // Saída estimada
  float ey[2] = {y[0] - y_hat[0], y[1] - y_hat[1]}; // Erro de medição
  
  // Atualização do estado estimado
  for (int i = 0; i < 4; i++) {
    x_hat[i] = x_hat_pred[i] + Ld[i][0]*ey[0] + Ld[i][1]*ey[1];
  }
}

float calcular_controle(float x, float x_dot, float phi, float phi_dot) {
  // Verifica se x está fora da faixa desejada
  if (x < -0.12f || x > 0.12f) {
    // Controle de retorno: força proporcional para trazer x de volta à faixa
    float K_return = 2000.0f;
    float u_return = K_return * x;
    return constrain(u_return, -MAX_PWM, MAX_PWM);
  }
  
  // Se estiver dentro da faixa, aplica o controle LQR normal
  float estados[4] = {x, x_dot, phi, phi_dot};
  float u = -(Kd[0] * estados[0] + Kd[1] * estados[1] + Kd[2] * estados[2] + Kd[3] * estados[3]);
  return constrain(u, -MAX_PWM, MAX_PWM);
}

void setup() {
  Serial.begin(115200);

  // Inicializa encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leEncoder, CHANGE);

  // Inicializa sensores VL53L0X
  pinMode(LOX1_XSHUT_PIN, OUTPUT);
  pinMode(LOX2_XSHUT_PIN, OUTPUT);
  digitalWrite(LOX1_XSHUT_PIN, LOW);
  digitalWrite(LOX2_XSHUT_PIN, LOW);
  delay(50);

  digitalWrite(LOX1_XSHUT_PIN, HIGH);
  delay(50);
  if (!lox1.begin(0x30)) { Serial.println("Erro sensor 1"); while (1); }

  digitalWrite(LOX2_XSHUT_PIN, HIGH);
  delay(50);
  if (!lox2.begin(0x31)) { Serial.println("Erro sensor 2"); while (1); }

  lox1.startRangeContinuous();
  lox2.startRangeContinuous();

  // Inicializa motor
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH); digitalWrite(L_EN, HIGH);

  // Define PWM nos pinos com nova API
  ledcAttach(RPWM, freqPWM, resolution);
  ledcAttach(LPWM, freqPWM, resolution);

  Serial.println("Sistema iniciado!");
}

void loop() {
  static unsigned long t_anterior = 0;

  if (millis() - t_anterior >= 50) {
    t_anterior = millis();
    
    // Leitura dos sensores
    y[0] = ler_posicao_mm() / 1000.0; // mm → metros
    y[1] = ler_phi_rad();
    
    // Atualiza o observador
    atualizar_observador(u, y);
    
    // Controle usando os estados estimados
    u = calcular_controle(x_hat[0], x_hat[1], x_hat[2], x_hat[3]);

    // PWM com base no sinal de controle
    if (u >= 0) {
      ledcWrite(RPWM, (int)u);
      ledcWrite(LPWM, 0);
    } else {
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, (int)(-u));
    }

    // Debug
    timestamp = millis();
    Serial.print(timestamp); Serial.print(",");
    Serial.print(y[0], 4); Serial.print(",");       // x medido
    Serial.print(y[1], 4); Serial.print(",");       // phi medido
    Serial.print(u, 1); Serial.print(",");
    Serial.print(x_hat[1], 4); Serial.print(",");   // x_dot estimado
    Serial.println(x_hat[3], 4);                    // phi_dot estimado
  }
}
