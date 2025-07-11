# **Introdução**

O projeto apresentado consiste na implementação de um **pêndulo invertido**, desenvolvido como parte das atividades da disciplina **Modelagem e Controle de Sistemas II**. A estrutura base foi adaptada a partir de uma impressora comum, aproveitando seu motor embutido de **24V** para a movimentação do trilho. Além da impressora, foram utilizados os seguintes componentes:

| Componente                                      | Descrição                                 | Datasheet                                                                                                                                                                                                           |
| ----------------------------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ESP32                                           | Microcontrolador                          | [📄](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)                                                                                                                            |
| 2 Sensores de distância VL53L0X                 | Sensor de distância (posição do carrinho) | [📄](https://www.alldatasheet.com/view.jsp?Searchword=Vl53l0x%20Datasheet&gad_source=1&gad_campaignid=1432848463&gclid=CjwKCAjw3MXBBhAzEiwA0vLXQTzKcui1WLHXg-_vCA1itTCsSSLOXApv7Bhh_TEmkd0yjqiV-MBufRoCYBwQAvD_BwE) |
| Encoder Incremental 360 AB PNP (F56)            | Medição do ângulo                         | [📄](https://www.alldatasheet.com/view.jsp?Searchword=Mpu-6050%20datasheet&gad_source=1&gad_campaignid=163458844&gclid=CjwKCAjw3MXBBhAzEiwA0vLXQTH4CT-uLhW6-a2hkWFem5TBKgU2mwys2hFuboTLkVvxGFpHKglb2RoCXcMQAvD_BwE) |
| Driver de motor BTS7960                         | Driver para motor DC                      | [📄](https://www.alldatasheet.com/view.jsp?Searchword=Bts7960%20datasheet&gad_source=1&gad_campaignid=145732807&gclid=CjwKCAjwprjDBhBTEiwA1m1d0hARd2jc2kv0Bl5XEZjIOlE777TRUl6Reo8d-SP2JrrT4wIWn1QZaRoC7yUQAvD_BwE)  |
| Fonte 24V Bivolt - 10A - 240W                   | Alimentação do motor                      |                                                                                                                                                                                                                     |
| Motor do Carro de Impressão da Impressora Epson | Motor 24V                                 |                                                                                                                                                                                                                     |

A combinação desses componentes permite a estabilização do pêndulo invertido por meio de estratégias de controle em tempo real, explorando conceitos teóricos aplicados na disciplina que serão apresentados a seguir.
# **Objetivos do Projeto**

O principal objetivo deste projeto é **estabilizar um pêndulo invertido dentro de uma região linearizada**, mantendo sua oscilação dentro de um limite de **±10°** em relação ao ponto de equilíbrio vertical. Para isso, serão utilizadas **técnicas de identificação de sistemas**, especificamente modelos **ARX** (Auto Regressive with eXogenous input), **ARMAX** (Auto Regressive Moving Average with eXogenous input), Alocação de polos e LQR (Regulador Linear Quadrático) para:

1. **Identificar dinamicamente o sistema** a partir de dados experimentais (ARX / ARMAX), ou obter um modelo matemático que represente adequadamente o comportamento do pêndulo (Alocação de polos / LQR).
2. **Projetar uma estratégia de controle** com base no modelo estimado, garantindo estabilidade dentro da faixa linear.
3. **Validar experimentalmente** o desempenho do controlador, analisando:
	- Tempo de estabilização.
	- Robustez a perturbações externas.
	- Limitações do modelo linearizado (considerando a restrição de ±10°).
4. **Comparar a eficiência** dos modelos **ARX** e **ARMAX** na representação do sistema.

# **Código**
## Esp32
### Sensores de distância
``` cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Objetos para os sensores
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Pinos XSHUT (reset) para cada sensor
#define LOX1_XSHUT_PIN 26
#define LOX2_XSHUT_PIN 27

void setup() {
  Serial.begin(115200);
  
  // Aguarda a serial (opcional para placas com USB nativo)
  while (!Serial) {
    delay(10);
  }

  Serial.println("\nIniciando dois sensores VL53L0X");

  // Configura pinos XSHUT
  pinMode(LOX1_XSHUT_PIN, OUTPUT);
  pinMode(LOX2_XSHUT_PIN, OUTPUT);
  
  // Inicialmente desliga ambos os sensores
  digitalWrite(LOX1_XSHUT_PIN, LOW);
  digitalWrite(LOX2_XSHUT_PIN, LOW);
  delay(10);

  // Ativa e configura o Sensor 1
  digitalWrite(LOX1_XSHUT_PIN, HIGH);
  delay(10);
  
  if (!lox1.begin(0x30)) {  // Endereço I2C 0x30
    Serial.println(F("Falha no Sensor 1 (VL53L0X)"));
    while (1);
  }
  Serial.println(F("Sensor 1 OK"));

  // Ativa e configura o Sensor 2
  digitalWrite(LOX2_XSHUT_PIN, HIGH);
  delay(10);
  
  if (!lox2.begin(0x31)) {  // Endereço I2C 0x31
    Serial.println(F("Falha no Sensor 2 (VL53L0X)"));
    while (1);
  }
  Serial.println(F("Sensor 2 OK"));

  // Inicia medição contínua
  lox1.startRangeContinuous();
  lox2.startRangeContinuous();

  Serial.println("\nPronto para medir...");
  Serial.println("------------------------");
}

void loop() {
  VL53L0X_RangingMeasurementData_t medida1;
  VL53L0X_RangingMeasurementData_t medida2;

  // Lê Sensor 1
  lox1.rangingTest(&medida1, false); // 'false' desativa debug no serial
  
  Serial.print("Sensor 1: ");
  if (medida1.RangeStatus != 4) {  // Status 4 = fora de alcance
    Serial.print(medida1.RangeMilliMeter);
    Serial.print("mm");
  } else {
    Serial.print("fora de alcance");
  }

  // Lê Sensor 2
  lox2.rangingTest(&medida2, false);
  
  Serial.print("  |  Sensor 2: ");
  if (medida2.RangeStatus != 4) {
    Serial.print(medida2.RangeMilliMeter);
    Serial.print("mm");
  } else {
    Serial.print("fora de alcance");
  }

  Serial.println(); // Nova linha
  delay(100); // Intervalo entre leituras
}
```
**Conexões**

| VL53L0X | ESP32                                   |
| ------- | --------------------------------------- |
| VCC     | 3.3V                                    |
| GND     | GND                                     |
| SDA     | GPIO 21                                 |
| SCL     | GPIO 22                                 |
| XSHUT   | GPIO 26 / GPIO 27 (Sensor 1 / Sensor 2) |

### Ponte H
``` cpp
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
```
**Conexões**

| BTS7960 |  ESP32  |
| ------- | :-----: |
| VCC     |  3.3V   |
| GND     |   GND   |
| RPWM    | GPIO 23 |
| LPWM    | GPIO 18 |
| R_EN    | GPIO 19 |
| L_EN    | GPIO 4  |

### Encoder
``` cpp
// Pinos do encoder
#define ENCODER_A 34
#define ENCODER_B 35

// Variáveis globais voláteis
volatile int contador = 0;
volatile int direcao = 0; // 0 = parado, 1 = horário, -1 = anti-horário
volatile unsigned long ultimaInterrupcao = 0;
const unsigned long debounceTime = 200; // tempo anti-rebote em microssegundos

// Variáveis para monitoramento
unsigned long ultimaLeitura = 0;
float velocidade = 0;

// Função de interrupção melhorada
void IRAM_ATTR leEncoder() {
  unsigned long tempoAtual = micros();
  
  // Filtro de debounce
  if (tempoAtual - ultimaInterrupcao < debounceTime) return;
  ultimaInterrupcao = tempoAtual;

  static int ultimoEstadoA = LOW;
  int estadoAtualA = digitalRead(ENCODER_A);
  
  if (estadoAtualA != ultimoEstadoA) {
    int estadoB = digitalRead(ENCODER_B);
    
    if (estadoB != estadoAtualA) {
      contador++;
      direcao = 1;
    } else {
      contador--;
      direcao = -1;
    }
    ultimoEstadoA = estadoAtualA;
  }
}

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leEncoder, CHANGE);
  
  Serial.begin(115200);
  Serial.println("Detector de Pequenos Movimentos do Encoder");
}

void loop() {
  // Atualiza a cada 100ms
  if (millis() - ultimaLeitura >= 100) {
    noInterrupts();
    int contadorAtual = contador;
    int dirAtual = direcao;
    interrupts();
    
    Serial.print("Pulsos: ");
    Serial.print(contadorAtual);
    Serial.print(" | Direção: ");
    Serial.println(dirAtual == 1 ? "Horário" : (dirAtual == -1 ? "Anti-Horário" : "Parado"));
    
    ultimaLeitura = millis();
  }
  
  delay(10);
}
```
**Conexões**

| Encoder PNP      |  ESP32  |
| ---------------- | :-----: |
| Vermelho (Vcc +) |   5V    |
| Preto (V0/GND)   |   GND   |
| Verde (Fase A)   | GPIO 34 |
| Branco (Fase B)  | GPIO 35 |

----

## Controle do Pendulo
A partir dos códigos abordados anteriormente criamos um código completo de captura de variáveis do modelo e utilizando de referencia o modelo matemático do [Control Tutorials for MATLAB & SIMULINK](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling) e dos parâmetros LQR do repositório no GitHub que usamos de referencia e a partir dessas variáveis criamos o [[Controle_Pendulo.ino]] e conferíamos os resultados através do [[coletor_serial.py]].

# Referências
KISHAN, I. **Inverted Pendulum**. GitHub, [S. l.], 2025. Disponível em: [https://github.com/imkishan96/Inverted_Pendulum/tree/master](https://github.com/imkishan96/Inverted_Pendulum/tree/master). Acesso em: 3 jun. 2025.

UNIVERSITY OF MICHIGAN. **Control Tutorials for MATLAB and Simulink: Inverted Pendulum - System Modeling**. [S. l.], 2025. Disponível em: [http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum§ion=SystemModeling](http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum%C2%A7ion=SystemModeling). Acesso em: 3 jun. 2025.
