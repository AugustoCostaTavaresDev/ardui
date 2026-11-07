// =====================================================================
//  SEGUIDOR DE BORDA - pista PRETA, borda BRANCA (Arduino Uno + L293D)
//  Segue a borda com UM sensor lateral (PD). Os outros dois só protegem.
//  Sem tempos calibrados, sem ré, sem memória de último lado.
// =====================================================================
#include <AFMotor.h>

#define LARGADA_AUTO       // sai sozinho 1 s depois de ligar (ignora o A3); comente p/ largar pela mão
//#define DEBUG            // descomente para imprimir no Serial (115200)

// ---------- 1) AJUSTES (só estes 6) ----------
enum { ESQUERDA, DIREITA };
const int   LADO_SEGUE = ESQUERDA; // sensor lateral que segue a borda (ESQUERDA ou DIREITA)
const float KP      = 40.0;  // correção proporcional: maior = volta mais rápido à borda, mas oscila
const float KD      = 1.5;   // amortecimento: maior = menos oscilação, mas amplifica ruído
const float V_BASE  = 100.0; // velocidade nas retas (%)
const float V_CURVA = 50.0;  // velocidade ao perder a borda e nas manobras fortes (%)
const float RIGOR   = 0.65;  // 0,55-0,85: brancura p/ contar "branco forte" (preto forte = 1-RIGOR); menor = reage mais cedo

// ---------- 2) CALIBRAÇÃO POR SENSOR (valores brutos do analogRead) ----------
// Leia com DEBUG ligado (campo "raw") e cole aqui. Maior valor = mais preto.
//                       E     C     D
const int BRANCO[3] = {  32,   28,   52 };  // sensor sobre a fita branca
const int PRETO[3]  = { 340,  317,  239 };  // sensor sobre o piso preto

// ---------- 3) FIXOS (não mexer) ----------
const uint8_t PIN_SENS[3] = { A4, A0, A5 }; // E, C, D
const uint8_t PIN_MAO = A3, PIN_LED = 13;
const int      LIM_MAO    = 300;     // analogRead acima disso = mão presente
const uint16_t T_LARGADA  = 0;       // ms entre tirar a mão e sair
const float    RAMPA      = 120.0;   // aceleração (%/s)
const float    ALFA_D     = 0.3;     // filtro da derivada
const float    BUSCA      = 0.25;    // sem fita sob o sensor seguido: curva suave p/ a borda (menor = mais reto)
const float    PERDIDO    = 0.08;    // brancura abaixo disso = sensor seguido só vê preto
const float    ALVO       = 0.5;     // meio da fita (mude p/ ~0,4 se a roda tocar a fita)
const uint32_t PERIODO_US = 10000;   // laço de 10 ms

enum { S_ESQ, S_CEN, S_DIR };                 // índices dos sensores
const uint8_t SEG = (LADO_SEGUE == ESQUERDA) ? S_ESQ : S_DIR;  // sensor que segue a borda
const uint8_t OPO = (LADO_SEGUE == ESQUERDA) ? S_DIR : S_ESQ;  // sensor do lado oposto

AF_DCMotor mEsq(4, MOTOR34_1KHZ);
AF_DCMotor mDir(2, MOTOR12_1KHZ);

enum { ESPERA, ARMADO, CORRE };
uint8_t estado = ESPERA;

int   raw[3];                        // leituras brutas
float b[3];                          // brancura: 0 = preto, 1 = branco
float vb = 0, eAnt = 0, dFilt = 0;   // velocidade base, erro anterior, derivada filtrada
float e = 0, u = 0, vEsq = 0, vDir = 0; // (também usados no DEBUG)
int   f = 0;                         // manobra forte: +1 longe da borda, -1 em direção à borda

// ---------- sensores ----------
void leSensores() {
  for (uint8_t i = 0; i < 3; i++) {
    analogRead(PIN_SENS[i]);                       // descarta a 1a leitura (multiplexador)
    long soma = 0;
    for (uint8_t k = 0; k < 8; k++) soma += analogRead(PIN_SENS[i]);
    raw[i] = soma / 8;
    int span = PRETO[i] - BRANCO[i];
    if (span < 1) span = 1;
    float v = (float)(PRETO[i] - raw[i]) / (float)span;   // normaliza por sensor
    b[i] = constrain(v, 0.0, 1.0);
  }
}

// calibração coerente e nenhum sensor em branco forte (só vale para armar)
bool sensoresOk() {
  for (uint8_t i = 0; i < 3; i++) {
    int span = PRETO[i] - BRANCO[i];
    if (span < 60) return false;                                         // sem contraste
    if (raw[i] < BRANCO[i] - span / 2 || raw[i] > PRETO[i] + span / 2) return false; // fora da faixa
    if (b[i] > RIGOR) return false;                                      // já está no branco
  }
  return true;
}

// ---------- motores (v em %, só para frente; 0 solta) ----------
void roda(AF_DCMotor &m, float v) {
  if (v > 100.0) v = 100.0;
  if (v <= 0.0) { m.setSpeed(0); m.run(RELEASE); return; }
  int pwm = (int)(v * 200.0 / 100.0 + 30);         // zona morta
  if (pwm > 255) pwm = 255;
  m.setSpeed(pwm);
  m.run(FORWARD);
}

void motores(float esq, float dir) {
  vEsq = esq; vDir = dir;
  roda(mEsq, esq);
  roda(mDir, dir);
}

// vs = roda do lado seguido, vo = roda do lado oposto
void motoresLado(float vs, float vo) {
  if (LADO_SEGUE == ESQUERDA) motores(vs, vo); else motores(vo, vs);
}

#ifdef DEBUG
void imprime() {
  Serial.print(F("a3 "));   Serial.print(analogRead(PIN_MAO)); Serial.print(' ');
  Serial.print(F("raw "));  Serial.print(raw[S_ESQ]); Serial.print(' ');
  Serial.print(raw[S_CEN]);    Serial.print(' ');     Serial.print(raw[S_DIR]);
  Serial.print(F(" b "));   Serial.print(b[S_ESQ], 2); Serial.print(' ');
  Serial.print(b[S_CEN], 2);   Serial.print(' ');     Serial.print(b[S_DIR], 2);
  Serial.print(F(" est ")); Serial.print(estado);
  Serial.print(F(" f "));   Serial.print(f);
  Serial.print(F(" e "));   Serial.print(e, 2);
  Serial.print(F(" u "));   Serial.print(u, 1);
  Serial.print(F(" vE "));  Serial.print(vEsq, 0);
  Serial.print(F(" vD "));  Serial.println(vDir, 0);
}
#endif

void setup() {
  pinMode(PIN_LED, OUTPUT);
  motores(0, 0);
#ifdef LARGADA_AUTO
  estado = ARMADO;                                 // conta T_LARGADA e sai
  digitalWrite(PIN_LED, HIGH);
#endif
#ifdef DEBUG
  Serial.begin(115200);
#endif
}

void loop() {
  static uint32_t tAnt = 0, tSolta = 0;
  uint32_t agora = micros();
  if (agora - tAnt < PERIODO_US) return;           // laço de 10 ms, sem delay()
  float dt = (agora - tAnt) * 1e-6;
  if (dt > 0.05) dt = 0.05;
  tAnt = agora;

  bool mao = analogRead(PIN_MAO) > LIM_MAO;
#ifdef LARGADA_AUTO
  mao = false;                                     // largada automática: sensor A3 ignorado
#endif
  leSensores();

  if (estado == ESPERA) {
    // ---- parado: mão + sensores ok = arma (LED fixo); LED piscando = sensores inválidos ----
    motores(0, 0);
    if (mao && sensoresOk()) { estado = ARMADO; tSolta = 0; digitalWrite(PIN_LED, HIGH); }
    else digitalWrite(PIN_LED, mao ? (millis() / 100) % 2 : LOW);

  } else if (estado == ARMADO) {
    // ---- armado: ao tirar a mão, espera 1 s e sai ----
    motores(0, 0);
    if (mao) tSolta = 0;
    else {
      if (tSolta == 0) tSolta = millis();
      if (millis() - tSolta >= T_LARGADA) {
        estado = CORRE;
        digitalWrite(PIN_LED, LOW);
        vb = 0; dFilt = 0; eAnt = (b[SEG] - ALVO) * 2.0;
      }
    }

  } else {
    // ---- correndo ----
    float bs = b[SEG], bo = b[OPO], bc = b[S_CEN];
    float eb = (bs - ALVO) * 2.0;                  // erro cru -1..+1; positivo = perto demais da fita
    e = eb;
    if (bs < PERDIDO && e < -BUSCA) e = -BUSCA;    // borda perdida: arco suave, não gira no lugar

    // Manobra forte (sem tempo): só termina quando o sensor deixa de ver branco forte.
    f = 0;
    if (bc > RIGOR)      f = (bs < 1.0 - RIGOR) ? -1 : +1; // frente branca: p/ longe da borda;
                                                            // se o seguido vê preto, a parede vem do lado oposto: p/ a borda
    else if (bs > RIGOR) f = +1;                 // seguido em cima da fita: afasta
    else if (bo > RIGOR) f = -1;                 // oposto em cima da fita: volta p/ a borda

    if (f != 0) {
      if (vb > V_CURVA) vb = V_CURVA;            // freia na hora
      motoresLado(f > 0 ? V_CURVA : 0, f > 0 ? 0 : V_CURVA); // gira em torno da roda de dentro
      eAnt = eb; dFilt = 0; u = 0;
    } else {
      // PD normal sobre o sensor seguido
      float vAlvo = (bs < 1.0 - RIGOR) ? V_CURVA : V_BASE;   // borda sumiu: reduz
      vb += RAMPA * dt;                          // acelera em rampa...
      if (vb > vAlvo) vb = vAlvo;                // ...e freia na hora
      dFilt += ALFA_D * ((eb - eAnt) / dt - dFilt);   // derivada do erro cru (sem salto)
      eAnt = eb;
      u = KP * e + KD * dFilt;                   // positivo = vira para longe da borda
      float vs = vb + u, vo = vb - u;            // roda do lado seguido acelera p/ afastar
      float ex = max(vs, vo) - 100.0;            // satura preservando a diferença
      if (ex > 0) { vs -= ex; vo -= ex; }
      if (vs < 0) vs = 0;                        // nunca ré
      if (vo < 0) vo = 0;
      motoresLado(vs, vo);
    }
  }

#ifdef DEBUG
  static uint8_t nd = 0;
  if (++nd >= 10) { nd = 0; imprime(); }           // a cada 100 ms
#endif
}
