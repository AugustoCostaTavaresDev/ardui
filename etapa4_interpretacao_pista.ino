// ETAPA 4 - Interpretacao da pista (SEM MOTORES: so le, calcula e imprime)
//
// Geometria: valores NOMINAIS do modelo chassi_n20_v8.scad.
// Origem = ponto medio do eixo das rodas. +X = direita, +Y = frente (mm).
// Centro optico de cada TCRT5000, olhando reto para baixo (7 mm do chao).
// Sensores de pista: indice 0 = E (A4), 1 = C, 2 = D (A5). Pino do C: ver PIN_C.
//
// Polaridade (medida pelo usuario): PRETO = valor ALTO, BRANCO = valor BAIXO.
// Pista: robo anda sobre o PRETO; o BRANCO e a borda (limite).

// O teste do usuario em 06/11 leu o central em A0 (nas etapas 2 e 3 era A1).
// Se o modulo voltar ao A1, troque so PIN_C.
const uint8_t PIN_E = A4;
const uint8_t PIN_C = A0;
const uint8_t PIN_D = A5;
const uint8_t PINOS[3] = { PIN_E, PIN_C, PIN_D };
const char    NOMES[3] = { 'E', 'C', 'D' };

const float POS_X[3] = { -19.2, 0.0, 19.2 };
const float POS_Y[3] = {  39.5, 57.7, 39.5 };
const float X_REF    = 19.2;   // braco lateral dos sensores E/D, normaliza LADO para -1 / 0 / +1

// Classificacao P/B por sensor, pela brancura calibrada (0..1000). A faixa bruta de cada sensor muda
// com a superficie (a lona reduz o contraste), entao limiares fixos em valor bruto deixam de valer.
const int16_t W_BRANCO = 650;   // brancura >= 650: branco
const int16_t W_PRETO  = 350;   // brancura <= 350: preto. Entre os dois, mantem o estado (histerese)

const uint8_t  N_AMOSTRAS = 8;
const uint16_t PERIODO_MS = 50;
const uint16_t SPAN_MIN   = 60;    // separacao minima preto-branco para normalizar (ruido medido: +-1)

uint16_t branco[3], preto[3];
bool temBranco = false, temPreto = false;
uint8_t estado[3] = { 255, 255, 255 };   // 0 branco, 1 preto, 255 desconhecido
float lado[3];                           // peso lateral de cada sensor, vem da geometria

uint16_t leSensor(uint8_t pino) {
  analogRead(pino);   // descartada: assenta o multiplexador do ADC
  uint32_t soma = 0;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) soma += analogRead(pino);
  return soma / N_AMOSTRAS;
}

void captura(uint16_t destino[3]) {   // media de 1 s
  const uint8_t N = 20;
  uint32_t soma[3] = { 0, 0, 0 };
  for (uint8_t k = 0; k < N; k++) {
    for (uint8_t i = 0; i < 3; i++) soma[i] += leSensor(PINOS[i]);
    delay(PERIODO_MS);
  }
  for (uint8_t i = 0; i < 3; i++) destino[i] = soma[i] / N;
}

void mostraConstantes() {
  Serial.println(F("--- copie para o codigo (ordem E, C, D) ---"));
  Serial.print(F("const uint16_t BRANCO[3] = { "));
  Serial.print(branco[0]); Serial.print(F(", ")); Serial.print(branco[1]); Serial.print(F(", ")); Serial.print(branco[2]);
  Serial.println(F(" };"));
  Serial.print(F("const uint16_t PRETO[3]  = { "));
  Serial.print(preto[0]); Serial.print(F(", ")); Serial.print(preto[1]); Serial.print(F(", ")); Serial.print(preto[2]);
  Serial.println(F(" };"));
  for (uint8_t i = 0; i < 3; i++) {
    int16_t span = (int16_t)preto[i] - (int16_t)branco[i];
    Serial.print(F("separacao ")); Serial.print(NOMES[i]); Serial.print('='); Serial.println(span);
    if (span < (int16_t)SPAN_MIN) {
      Serial.print(F("AVISO: sensor ")); Serial.print(NOMES[i]);
      Serial.println(F(" com separacao baixa: sem normalizacao"));
    }
  }
}

// whiteness 0..1000 (0 = preto/seguro, 1000 = branco/borda); -1 se nao calibrado ou separacao pequena
int16_t brancura(uint8_t i, uint16_t v) {
  if (!(temBranco && temPreto)) return -1;
  int32_t span = (int32_t)preto[i] - (int32_t)branco[i];
  if (span < (int32_t)SPAN_MIN) return -1;
  int32_t n = ((int32_t)v - (int32_t)branco[i]) * 1000L / span;   // 0 = branco, 1000 = preto
  if (n < 0) n = 0;
  if (n > 1000) n = 1000;
  return (int16_t)(1000 - n);
}

void atualizaEstado(uint8_t i, int16_t w) {
  if (w < 0) { estado[i] = 255; return; }   // sem calibracao valida: desconhecido (modo conservador)
  if (w >= W_BRANCO)     estado[i] = 0;
  else if (w <= W_PRETO) estado[i] = 1;
  // entre os limiares: mantem o estado anterior
}

// Diagnostico para log e para o modo conservador. Os valores continuos (erro, frontal, risco)
// e que alimentam o controle; este rotulo so descreve a situacao.
const __FlashStringHelper* diagnostico() {
  for (uint8_t i = 0; i < 3; i++) if (estado[i] == 255) return F("DESCONHECIDO");
  uint8_t m = (estado[0] == 0 ? 1 : 0) | (estado[1] == 0 ? 2 : 0) | (estado[2] == 0 ? 4 : 0);   // E=1 C=2 D=4: bit = ve branco
  switch (m) {
    case 0: return F("OK");
    case 1: return F("BORDA_ESQ");
    case 4: return F("BORDA_DIR");
    case 2: return F("FRONTAL");
    case 3: return F("FRONTAL_ESQ");
    case 6: return F("FRONTAL_DIR");
    case 5: return F("INCONSISTENTE");   // branco nos dois lados e preto na frente: fisicamente improvavel
    default: return F("FORA");           // os tres no branco: levantado ou fora da pista
  }
}

void mostraGeometria() {
  Serial.println(F("Geometria (mm, origem no meio do eixo): sensor x y raio angulo"));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(NOMES[i]); Serial.print(' ');
    Serial.print(POS_X[i], 1); Serial.print(' '); Serial.print(POS_Y[i], 1); Serial.print(' ');
    Serial.print(sqrt(POS_X[i] * POS_X[i] + POS_Y[i] * POS_Y[i]), 1); Serial.print(' ');
    Serial.println(atan2(POS_X[i], POS_Y[i]) * 57.2958, 1);
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) lado[i] = POS_X[i] / X_REF;   // E=-1, C=0, D=+1
  Serial.println(F("Comandos: b = captura BRANCO, p = captura PRETO, g = mostra geometria"));
  Serial.println(F("Linha: ms | brancura E C D (0-1000) | erro | frontal | risco | diagnostico"));
  Serial.println(F("erro > 0: branco a DIREITA (corrigir para a esquerda). frontal = brancura de C."));
  mostraGeometria();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'b') { captura(branco); temBranco = true; if (temPreto) mostraConstantes(); }
    if (c == 'p') { captura(preto);  temPreto  = true; if (temBranco) mostraConstantes(); }
    if (c == 'g') mostraGeometria();
  }

  static uint32_t tAnterior = 0;
  if (millis() - tAnterior < PERIODO_MS) return;
  tAnterior = millis();

  int16_t w[3];
  bool valido = true;
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t v = leSensor(PINOS[i]);
    w[i] = brancura(i, v);
    atualizaEstado(i, w[i]);
    if (w[i] < 0) valido = false;
  }

  Serial.print(millis()); Serial.print(F(" | "));
  if (valido) {
    float erro = 0;
    int16_t risco = 0;
    for (uint8_t i = 0; i < 3; i++) {
      erro += lado[i] * w[i];          // lateral: so E e D tem braco lateral; C nao da direcao
      if (w[i] > risco) risco = w[i];
    }
    for (uint8_t i = 0; i < 3; i++) { Serial.print(w[i]); Serial.print(' '); }
    Serial.print(F("| ")); Serial.print((int16_t)erro);
    Serial.print(F(" | ")); Serial.print(w[1]);
    Serial.print(F(" | ")); Serial.print(risco);
  } else {
    Serial.print(F("-- -- -- | -- | -- | --   (faltam b e p)"));
  }
  Serial.print(F(" | ")); Serial.println(diagnostico());
}
