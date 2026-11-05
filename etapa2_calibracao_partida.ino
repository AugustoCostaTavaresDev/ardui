// ETAPA 2 - Calibracao, classificacao preto/branco e deteccao de partida
// Sem motores. Apenas le, classifica e imprime.
// Pinos confirmados: central A1, esquerdo A4, direito A5, partida A3.
// Polaridade medida pelo usuario: PRETO = valor ALTO, BRANCO = valor BAIXO.

const uint8_t PIN_IR_C = A1;
const uint8_t PIN_IR_E = A4;
const uint8_t PIN_IR_D = A5;
const uint8_t PIN_IR_S = A3;

// Indices dos sensores de pista: 0 = E, 1 = C, 2 = D
const uint8_t PINOS_PISTA[3] = { PIN_IR_E, PIN_IR_C, PIN_IR_D };
const char    NOMES[3]       = { 'E', 'C', 'D' };

// Limiares informados. Entre os dois, o estado anterior e mantido (histerese).
const uint16_t LIM_PRETO  = 150;  // acima disso: preto
const uint16_t LIM_BRANCO = 120;  // abaixo disso: branco

// Partida: sensor apontado para cima, repouso ~950, a mao faz o valor cair.
const uint16_t LIM_ARMAR   = 900; // so arma se ficar acima disso
const uint16_t LIM_PARTIDA = 870; // abaixo disso (confirmado): partiu
const uint8_t  CONFIRMA    = 3;   // leituras seguidas (3 x 50 ms = 150 ms)

const uint8_t  N_AMOSTRAS = 8;
const uint16_t PERIODO_MS = 50;
const uint16_t SPAN_MIN   = 150;  // diferenca minima preto-branco para normalizar

uint16_t branco[3], preto[3];
bool temBranco = false, temPreto = false;

enum Fase : uint8_t { ARMANDO, ESPERA, PARTIU };
Fase fase = ARMANDO;
uint8_t contArma = 0, contPartida = 0;

// 0 = branco, 1 = preto, 255 = desconhecido (ainda nao saiu da faixa de histerese)
uint8_t estado[3] = { 255, 255, 255 };

uint16_t leSensor(uint8_t pino) {
  analogRead(pino);  // descartada: assenta o multiplexador do ADC
  uint32_t soma = 0;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) soma += analogRead(pino);
  return soma / N_AMOSTRAS;
}

// Media de 1 s (20 leituras) de cada sensor de pista
void captura(uint16_t destino[3]) {
  const uint8_t N = 20;
  uint32_t soma[3] = { 0, 0, 0 };
  for (uint8_t k = 0; k < N; k++) {
    for (uint8_t i = 0; i < 3; i++) soma[i] += leSensor(PINOS_PISTA[i]);
    delay(PERIODO_MS);
  }
  for (uint8_t i = 0; i < 3; i++) destino[i] = soma[i] / N;
}

void mostraCaptura(const __FlashStringHelper* nome, uint16_t v[3]) {
  Serial.print(nome);
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("  ")); Serial.print(NOMES[i]); Serial.print('='); Serial.print(v[i]);
  }
  Serial.println();
}

void mostraConstantes() {
  if (!(temBranco && temPreto)) return;
  Serial.println(F("--- copie para o codigo (ordem E, C, D) ---"));
  Serial.print(F("const uint16_t BRANCO[3] = { "));
  Serial.print(branco[0]); Serial.print(F(", ")); Serial.print(branco[1]); Serial.print(F(", ")); Serial.print(branco[2]);
  Serial.println(F(" };"));
  Serial.print(F("const uint16_t PRETO[3]  = { "));
  Serial.print(preto[0]); Serial.print(F(", ")); Serial.print(preto[1]); Serial.print(F(", ")); Serial.print(preto[2]);
  Serial.println(F(" };"));
  for (uint8_t i = 0; i < 3; i++) {
    int16_t span = (int16_t)preto[i] - (int16_t)branco[i];
    if (span < (int16_t)SPAN_MIN) {
      Serial.print(F("AVISO: sensor ")); Serial.print(NOMES[i]);
      Serial.print(F(" com separacao ")); Serial.print(span);
      Serial.println(F(" (< 150): nao sera normalizado"));
    }
  }
}

// 0..1000 (0 = branco, 1000 = preto); -1 se nao calibrado ou separacao pequena
int16_t normaliza(uint8_t i, uint16_t v) {
  if (!(temBranco && temPreto)) return -1;
  int32_t span = (int32_t)preto[i] - (int32_t)branco[i];
  if (span < (int32_t)SPAN_MIN) return -1;
  int32_t n = ((int32_t)v - (int32_t)branco[i]) * 1000L / span;
  if (n < 0) n = 0;
  if (n > 1000) n = 1000;
  return (int16_t)n;
}

void atualizaEstado(uint8_t i, uint16_t v) {
  if (v > LIM_PRETO)       estado[i] = 1;
  else if (v < LIM_BRANCO) estado[i] = 0;
  // entre os limiares: mantem o estado anterior
}

void atualizaPartida(uint16_t s) {
  if (fase == ARMANDO) {
    if (s >= LIM_ARMAR) { if (++contArma >= CONFIRMA) { fase = ESPERA; contPartida = 0; } }
    else contArma = 0;
  } else if (fase == ESPERA) {
    if (s < LIM_PARTIDA) { if (++contPartida >= CONFIRMA) fase = PARTIU; }
    else contPartida = 0;
  }
  digitalWrite(LED_BUILTIN, fase == PARTIU ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(F("Comandos: b = captura BRANCO, p = captura PRETO, r = rearma partida"));
  Serial.println(F("Linha: ms | E C D (bruto) | n (0-1000) | pista (P/B/?) | S fase"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b') { captura(branco); temBranco = true; mostraCaptura(F("BRANCO"), branco); mostraConstantes(); }
    if (c == 'p') { captura(preto);  temPreto  = true; mostraCaptura(F("PRETO "), preto);  mostraConstantes(); }
    if (c == 'r') { fase = ARMANDO; contArma = 0; contPartida = 0; }
  }

  static uint32_t tAnterior = 0;
  if (millis() - tAnterior < PERIODO_MS) return;
  tAnterior = millis();

  uint16_t v[3];
  for (uint8_t i = 0; i < 3; i++) {
    v[i] = leSensor(PINOS_PISTA[i]);
    atualizaEstado(i, v[i]);
  }
  uint16_t s = leSensor(PIN_IR_S);
  atualizaPartida(s);

  Serial.print(millis()); Serial.print(F(" | "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(v[i]); Serial.print(' '); }
  Serial.print(F("| "));
  for (uint8_t i = 0; i < 3; i++) {
    int16_t n = normaliza(i, v[i]);
    if (n < 0) Serial.print(F("-- ")); else { Serial.print(n); Serial.print(' '); }
  }
  Serial.print(F("| "));
  for (uint8_t i = 0; i < 3; i++) Serial.print(estado[i] == 1 ? 'P' : (estado[i] == 0 ? 'B' : '?'));
  Serial.print(F(" | ")); Serial.print(s); Serial.print(' ');
  Serial.println(fase == ARMANDO ? F("ARMANDO") : (fase == ESPERA ? F("ESPERA") : F("PARTIU")));
}
