// ETAPA 1 - Leitura bruta dos sensores IR (analogico)
// Sem motores. Apenas mede e imprime.
// Pinos confirmados: central A1, esquerdo A4, direito A5, partida A3.

const uint8_t PIN_IR_C = A1;  // central dianteiro (pista)
const uint8_t PIN_IR_E = A4;  // esquerdo (pista)
const uint8_t PIN_IR_D = A5;  // direito (pista)
const uint8_t PIN_IR_S = A3;  // partida (mao por cima)

const uint8_t  N_SENS     = 4;    // E, C, D, S
const uint8_t  N_AMOSTRAS = 8;    // media por leitura
const uint16_t PERIODO_MS = 50;   // 20 leituras por segundo

const uint8_t PINOS[N_SENS] = { PIN_IR_E, PIN_IR_C, PIN_IR_D, PIN_IR_S };
const char    NOMES[N_SENS] = { 'E', 'C', 'D', 'S' };

uint16_t minV[N_SENS];
uint16_t maxV[N_SENS];

uint16_t leSensor(uint8_t pino) {
  analogRead(pino);  // leitura descartada: assenta o multiplexador do ADC
  uint32_t soma = 0;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) soma += analogRead(pino);
  return soma / N_AMOSTRAS;
}

void zeraExtremos() {
  for (uint8_t i = 0; i < N_SENS; i++) { minV[i] = 1023; maxV[i] = 0; }
}

void imprimeExtremos() {
  Serial.print(F("MIN/MAX"));
  for (uint8_t i = 0; i < N_SENS; i++) {
    Serial.print(F("  ")); Serial.print(NOMES[i]); Serial.print(':');
    Serial.print(minV[i]); Serial.print('/'); Serial.print(maxV[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  zeraExtremos();
  Serial.println(F("Comandos: r = zera min/max, m = mostra min/max"));
  Serial.println(F("ms,E_A4,C_A1,D_A5,S_A3"));
}

void loop() {
  static uint32_t tAnterior = 0;
  if (millis() - tAnterior < PERIODO_MS) return;
  tAnterior = millis();

  uint16_t v[N_SENS];
  for (uint8_t i = 0; i < N_SENS; i++) {
    v[i] = leSensor(PINOS[i]);
    if (v[i] < minV[i]) minV[i] = v[i];
    if (v[i] > maxV[i]) maxV[i] = v[i];
  }

  Serial.print(millis());
  for (uint8_t i = 0; i < N_SENS; i++) { Serial.print(','); Serial.print(v[i]); }
  Serial.println();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') zeraExtremos();
    if (c == 'm') imprimeExtremos();
  }
}
