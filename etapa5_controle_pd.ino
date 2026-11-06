// ETAPA 5 - Controle PD de direcao + motores
// TESTE NO CHAO, EM BAIXA VELOCIDADE, COM O CABO USB E A MAO PERTO DO ROBO.
//
// Entradas: leitura e interpretacao da etapa 4 (brancura de cada sensor, diagnostico).
// Saida: velocidade de cada motor, 0..100 (100 = PWM_MAX). Esta etapa NAO da re.
//
// Convencao: erro > 0 = branco a DIREITA -> virar a ESQUERDA (roda direita mais rapida).
// Esta etapa trata aproximacoes laterais da borda. Qualquer situacao ambigua ou inconsistente
// (so o central no branco, branco dos dois lados, tres no branco, sem calibracao) PARA o robo:
// comportamento conservador. A recuperacao de verdade fica para a etapa 7.
//
// Comandos (Serial 115200; "Nova-linha" no monitor nao atrapalha):
//   b = captura BRANCO    p = captura PRETO     (so com o robo parado)
//   g = largar: espera 1 s e so larga se estiver calibrado e com os 3 sensores no preto
//   x = parar
//   1 / 2 = Kp -/+ 5     3 / 4 = Kd -/+ 0,5     5 / 6 = velocidade base -/+ 5

#include <AFMotor.h>

// ---------- Pinos ----------
const uint8_t PIN_E = A4;
const uint8_t PIN_C = A0;   // o teste de 06/11 leu o central em A0 (nas etapas 2 e 3 era A1)
const uint8_t PIN_D = A5;
const uint8_t PINOS[3] = { PIN_E, PIN_C, PIN_D };
const char    NOMES[3] = { 'E', 'C', 'D' };

// Posicao lateral de cada sensor (mm, modelo chassi_n20_v8.scad). O central (X = 0) nao da direcao.
const float POS_X[3] = { -19.2, 0.0, 19.2 };
const float X_REF    = 19.2;
float lado[3];                         // E = -1, C = 0, D = +1

// ---------- Interpretacao (igual a etapa 4) ----------
const int16_t  W_BRANCO   = 650;       // brancura >= 650: branco
const int16_t  W_PRETO    = 350;       // brancura <= 350: preto; entre os dois mantem o estado
const uint16_t SPAN_MIN   = 60;
const uint8_t  N_AMOSTRAS = 8;

// ---------- Controle (valores PROVISORIOS: ajustar no teste) ----------
float kp = 40.0f;                      // velocidade por unidade de erro normalizado (-1..1)
float kd = 1.5f;                       // velocidade por (unidade de erro / s)
float vBase = 35.0f;                   // velocidade base alvo (0..100)
const float    V_BASE_MAX = 60.0f;     // teto de seguranca desta etapa
const float    V_RAMPA    = 100.0f;    // aceleracao da base, unidades/s (0 -> 35 em 0,35 s)
const float    ALFA_D     = 0.3f;      // filtro da derivada (0..1; menor = mais filtro)
const uint32_t PERIODO_US = 10000;     // ciclo de controle: 10 ms
const uint16_t T_ARMA_MS  = 1000;      // espera entre o comando g e a largada
const uint32_t T_MAX_MS   = 10000;     // limite de duracao de uma corrida de teste
const uint8_t  N_PARADA   = 3;         // ciclos seguidos em condicao de parada (30 ms)

// ---------- Motores (esquerdo = M4, direito = M2) ----------
const uint8_t PWM_MAX     = 200;       // decisao do usuario
const bool    INVERTE_ESQ = false;
const bool    INVERTE_DIR = false;
AF_DCMotor mEsq(4, MOTOR34_1KHZ);
AF_DCMotor mDir(2, MOTOR12_1KHZ);

uint16_t branco[3], preto[3];
bool temBranco = false, temPreto = false;
uint8_t estado[3] = { 255, 255, 255 }; // 0 branco, 1 preto, 255 desconhecido

uint16_t leSensor(uint8_t pino) {
  analogRead(pino);   // descartada: assenta o multiplexador do ADC
  uint32_t soma = 0;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) soma += analogRead(pino);
  return soma / N_AMOSTRAS;
}

void captura(uint16_t destino[3]) {   // media de 1 s, bloqueante: so com o robo parado
  const uint8_t N = 20;
  uint32_t soma[3] = { 0, 0, 0 };
  for (uint8_t k = 0; k < N; k++) {
    for (uint8_t i = 0; i < 3; i++) soma[i] += leSensor(PINOS[i]);
    delay(50);
  }
  for (uint8_t i = 0; i < 3; i++) destino[i] = soma[i] / N;
}

void mostraCalibracao() {
  Serial.print(F("BRANCO"));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(' '); Serial.print(branco[i]); }
  Serial.println();
  Serial.print(F("PRETO "));
  for (uint8_t i = 0; i < 3; i++) { Serial.print(' '); Serial.print(preto[i]); }
  Serial.println();
  for (uint8_t i = 0; i < 3; i++) {
    int16_t span = (int16_t)preto[i] - (int16_t)branco[i];
    Serial.print(F("separacao ")); Serial.print(NOMES[i]); Serial.print('='); Serial.println(span);
    if (span < (int16_t)SPAN_MIN) {
      Serial.print(F("AVISO: sensor ")); Serial.print(NOMES[i]);
      Serial.println(F(" com separacao baixa: sem normalizacao"));
    }
  }
}

// 0..1000 (0 = preto/seguro, 1000 = branco/borda); -1 se nao calibrado ou separacao pequena
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
  if (w < 0) { estado[i] = 255; return; }   // sem calibracao valida: desconhecido
  if (w >= W_BRANCO)     estado[i] = 0;
  else if (w <= W_PRETO) estado[i] = 1;
  // entre os limiares: mantem o estado anterior
}

enum Diag : uint8_t { D_DESCONHECIDO, D_OK, D_BORDA_ESQ, D_BORDA_DIR, D_FRONTAL,
                      D_FRONTAL_ESQ, D_FRONTAL_DIR, D_INCONSISTENTE, D_FORA };

Diag classifica() {
  for (uint8_t i = 0; i < 3; i++) if (estado[i] == 255) return D_DESCONHECIDO;
  uint8_t m = (estado[0] == 0 ? 1 : 0) | (estado[1] == 0 ? 2 : 0) | (estado[2] == 0 ? 4 : 0);   // E=1 C=2 D=4
  switch (m) {
    case 0: return D_OK;
    case 1: return D_BORDA_ESQ;
    case 4: return D_BORDA_DIR;
    case 2: return D_FRONTAL;
    case 3: return D_FRONTAL_ESQ;
    case 6: return D_FRONTAL_DIR;
    case 5: return D_INCONSISTENTE;   // branco nos dois lados e preto na frente
    default: return D_FORA;           // os tres no branco
  }
}

const __FlashStringHelper* nomeDiag(Diag d) {
  switch (d) {
    case D_DESCONHECIDO:  return F("DESCONHECIDO");
    case D_OK:            return F("OK");
    case D_BORDA_ESQ:     return F("BORDA_ESQ");
    case D_BORDA_DIR:     return F("BORDA_DIR");
    case D_FRONTAL:       return F("FRONTAL");
    case D_FRONTAL_ESQ:   return F("FRONTAL_ESQ");
    case D_FRONTAL_DIR:   return F("FRONTAL_DIR");
    case D_INCONSISTENTE: return F("INCONSISTENTE");
    default:              return F("FORA");
  }
}

// ---------- Motores ----------
void setMotor(AF_DCMotor &m, float v, bool inverte) {
  if (v > 100.0f) v = 100.0f;
  if (v <= 0.0f) { m.setSpeed(0); m.run(RELEASE); return; }   // sem re nesta etapa
  m.setSpeed((uint8_t)(v * PWM_MAX / 100.0f));
  m.run(inverte ? BACKWARD : FORWARD);
}

void setMotores(float vE, float vD) {
  setMotor(mEsq, vE, INVERTE_ESQ);
  setMotor(mDir, vD, INVERTE_DIR);
}

// ---------- Estado do controle ----------
enum Ctl : uint8_t { PARADO, ARMADO, RODANDO };
Ctl ctl = PARADO;
uint32_t tArma = 0, tInicio = 0, maxCiclo = 0;
uint16_t atrasos = 0;
float vb = 0, eAnt = 0, dFilt = 0;
uint8_t contParada = 0, cicloLog = 0;
bool ultValido = false;
Diag ultDiag = D_DESCONHECIDO;

void parar(const __FlashStringHelper* motivo) {
  setMotores(0, 0);
  ctl = PARADO;
  Serial.print(F("PARADO: ")); Serial.print(motivo);
  Serial.print(F(" | t=")); Serial.print(millis() - tInicio);
  Serial.print(F(" ms | ciclo max=")); Serial.print(maxCiclo);
  Serial.print(F(" us | atrasos=")); Serial.println(atrasos);
}

void mostraParametros() {
  Serial.print(F("kp=")); Serial.print(kp, 1);
  Serial.print(F("  kd=")); Serial.print(kd, 1);
  Serial.print(F("  vBase=")); Serial.println(vBase, 0);
}

void iniciar() {
  if (ctl != PARADO) return;
  if (!ultValido) { Serial.println(F("NAO LARGOU: sem calibracao valida (use b e p)")); return; }
  if (ultDiag != D_OK) {
    Serial.print(F("NAO LARGOU: sensores nao estao todos no preto (")); Serial.print(nomeDiag(ultDiag)); Serial.println(')');
    return;
  }
  ctl = ARMADO;
  tArma = millis();
  mostraParametros();
  Serial.println(F("ARMADO: larga em 1 s. x para cancelar."));
}

void tratarSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') {
      if (ctl != PARADO) parar(F("comando x"));
    } else if (c == 'g') {
      iniciar();
    } else if (c == 'b' && ctl == PARADO) {
      captura(branco); temBranco = true; mostraCalibracao();
    } else if (c == 'p' && ctl == PARADO) {
      captura(preto); temPreto = true; mostraCalibracao();
    } else if (c >= '1' && c <= '6') {
      if (c == '1' && kp >= 5.0f)    kp -= 5.0f;
      if (c == '2' && kp <= 195.0f)  kp += 5.0f;
      if (c == '3' && kd >= 0.5f)    kd -= 0.5f;
      if (c == '4' && kd <= 19.5f)   kd += 0.5f;
      if (c == '5' && vBase >= 15.0f) vBase -= 5.0f;
      if (c == '6' && vBase <= V_BASE_MAX - 5.0f) vBase += 5.0f;
      mostraParametros();
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) lado[i] = POS_X[i] / X_REF;
  setMotores(0, 0);
  Serial.println(F("ETAPA 5 - PD. b = branco, p = preto (robo parado), g = largar, x = parar"));
  Serial.println(F("1/2 Kp-/+  3/4 Kd-/+  5/6 vBase-/+"));
  mostraParametros();
}

void loop() {
  tratarSerial();

  static uint32_t tAnt = 0;
  uint32_t agora = micros();
  if (agora - tAnt < PERIODO_US) return;
  float dt = (agora - tAnt) * 1e-6f;
  if (dt > 0.05f) dt = 0.05f;
  bool atrasado = (agora - tAnt) > PERIODO_US + 5000;
  tAnt = agora;

  // --- leitura e interpretacao ---
  int16_t w[3];
  bool valido = true;
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t v = leSensor(PINOS[i]);
    w[i] = brancura(i, v);
    atualizaEstado(i, w[i]);
    if (w[i] < 0) valido = false;
  }
  Diag d = classifica();
  float erro = 0;
  if (valido) for (uint8_t i = 0; i < 3; i++) erro += lado[i] * w[i];   // = wD - wE
  float e = erro / 1000.0f;
  ultValido = valido;
  ultDiag = d;

  // --- maquina de controle ---
  if (ctl == ARMADO) {
    if (d != D_OK) {
      ctl = PARADO;
      Serial.println(F("CANCELADO: sensores sairam do preto"));
    } else if (millis() - tArma >= T_ARMA_MS) {
      ctl = RODANDO;
      tInicio = millis();
      vb = 0; eAnt = e; dFilt = 0; contParada = 0; cicloLog = 0; maxCiclo = 0; atrasos = 0;
      Serial.println(F("LARGOU"));
      Serial.println(F("t_ms,wE,wC,wD,erro,u,vE,vD,diag"));
    }
  }

  if (ctl == RODANDO) {
    if (atrasado) atrasos++;
    bool parada = !valido || d == D_DESCONHECIDO || d == D_FRONTAL || d == D_INCONSISTENTE || d == D_FORA;
    contParada = parada ? contParada + 1 : 0;

    if (contParada >= N_PARADA) {
      parar(nomeDiag(d));
    } else if (millis() - tInicio > T_MAX_MS) {
      parar(F("tempo maximo"));
    } else {
      float de = (e - eAnt) / dt;
      eAnt = e;
      dFilt += ALFA_D * (de - dFilt);
      float u = kp * e + kd * dFilt;        // u > 0: branco a direita -> esquerda mais lenta
      vb += V_RAMPA * dt;
      if (vb > vBase) vb = vBase;
      float vE = vb - u, vD = vb + u;
      if (vE < 0) vE = 0;
      if (vE > 100) vE = 100;
      if (vD < 0) vD = 0;
      if (vD > 100) vD = 100;
      setMotores(vE, vD);

      if (++cicloLog >= 5) {                // log a cada 50 ms
        cicloLog = 0;
        Serial.print(millis() - tInicio); Serial.print(',');
        Serial.print(w[0]); Serial.print(','); Serial.print(w[1]); Serial.print(','); Serial.print(w[2]); Serial.print(',');
        Serial.print((int)erro); Serial.print(','); Serial.print((int)u); Serial.print(',');
        Serial.print((int)vE); Serial.print(','); Serial.print((int)vD); Serial.print(',');
        Serial.println(nomeDiag(d));
      }
    }
    uint32_t dur = micros() - agora;
    if (dur > maxCiclo) maxCiclo = dur;
  } else if (ctl == PARADO) {
    static uint32_t tStatus = 0;
    if (millis() - tStatus >= 500) {        // status a 2 Hz para posicionar o robo
      tStatus = millis();
      Serial.print(F("[parado] w=")); Serial.print(w[0]); Serial.print(' '); Serial.print(w[1]); Serial.print(' '); Serial.print(w[2]);
      Serial.print(F(" erro=")); Serial.print((int)erro); Serial.print(' '); Serial.println(nomeDiag(d));
    }
  }
}
