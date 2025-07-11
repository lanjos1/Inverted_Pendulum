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
/*
**Conexões**

| Encoder PNP      |  ESP32  |
| ---------------- | :-----: |
| Vermelho (Vcc +) |   5V    |
| Preto (V0/GND)   |   GND   |
| Verde (Fase A)   | GPIO 34 |
| Branco (Fase B)  | GPIO 35 |
*/
