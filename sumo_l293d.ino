/*
 * ROBÔ DE SUMÔ — Arduino + Shield L293D (biblioteca AFMotor)
 * Motores: M3 = direito | M2 = esquerdo
 * Sensores de linha: F_IR A1 | BL_IR A4 | BR_IR A5
 * Ultrassônico: TRIG A2 | ECHO A3
 *
 * Biblioteca: "Adafruit Motor Shield library" (AFMotor.h) — v1, a do shield L293D.
 *
 * COMO USAR
 * 1) MODO_CALIBRACAO 1 -> sobe, abre o Serial Monitor (115200). Robô NÃO anda.
 *    Passe cada IR sobre o preto e sobre o branco. Ele mostra valor atual,
 *    min/max vistos e um limiar sugerido (meio do caminho) pra cada sensor.
 * 2) Copie os limiares sugeridos pra tabela "ir[]" abaixo.
 * 3) MODO_CALIBRACAO 0 -> modo luta.
 */

#include <AFMotor.h>

// ======================= CONFIGURAÇÃO =======================

#define MODO_CALIBRACAO 1   // 1 = só lê e imprime sensores | 0 = luta
#define DEBUG_ESTADOS   0   // 1 = imprime troca de estado durante a luta

// Se o F_IR for sensor de OPONENTE (e não de linha), coloque 1.
// Aí ele vira gatilho de ataque instantâneo e a borda da frente não é checada.
#define F_IR_DETECTA_OPONENTE 0

// Motores invertidos? true inverte o sentido daquele motor.
const bool INVERTE_DIR = true;   // M3
const bool INVERTE_ESQ = true;   // M2

// Pinos do ultrassônico
const uint8_t PINO_TRIG = A2;
const uint8_t PINO_ECHO = A3;

// Sensores IR — limiar individual.
// brancoEhMenor = true  -> leitura ABAIXO do limiar = branco (maioria dos TCRT5000)
// brancoEhMenor = false -> leitura ACIMA do limiar = branco
struct SensorIR {
  uint8_t pino;
  int limiar;
  bool brancoEhMenor;
  const char* nome;
  int minV, maxV;   // usados na calibração
};

SensorIR ir[3] = {
  // pino, limiar, brancoEhMenor, nome
  { A1, 500, true, "F ", 1023, 0 },
  { A4, 500, true, "BL", 1023, 0 },
  { A5, 500, true, "BR", 1023, 0 },
};
enum { S_F = 0, S_BL = 1, S_BR = 2 };

// Velocidades (0–255)
const int VEL_MAX     = 255;  // ataque e fuga
const int VEL_BUSCA   = 170;  // giro de busca (rápido demais = ultrassom perde o alvo)
const int VEL_AVANCA  = 150;  // avanço curto quando a busca não acha nada
const int VEL_CURVA   = 140;  // roda interna na fuga pra frente (curva)

// Detecção do oponente
const int DIST_ATAQUE_CM = 60;  // ajuste ao tamanho da arena
const int DIST_MAX_CM    = 80;  // limite de leitura (define o timeout)
const unsigned long INTERVALO_US_MS = 35;  // entre pings do ultrassônico

// Tempos (ms) — CALIBRE TEMPO_90 olhando o robô girar em VEL_BUSCA
const unsigned long ESPERA_INICIO     = 5000; // regra dos 5 s
const unsigned long TEMPO_90          = 250;  // tempo pra girar ~90°
const unsigned long TEMPO_GIRO_MAX    = 2500; // giro contínuo antes de reposicionar
const unsigned long TEMPO_AVANCA      = 300;  // avanço de reposição
const unsigned long TEMPO_PERDA       = 300;  // segue empurrando se o ultrassom "piscar" no contato
const unsigned long TEMPO_RECUO       = 300;  // ré ao ver borda na frente
const unsigned long TEMPO_GIRO_FUGA   = 280;  // giro depois da ré (~135°)
const unsigned long TEMPO_FRENTE_FUGA = 250;  // avanço ao ver borda atrás

// ======================= HARDWARE =======================

AF_DCMotor motorDir(3);
AF_DCMotor motorEsq(2);

int velAtualEsq = 9999, velAtualDir = 9999;

void setMotor(AF_DCMotor &m, int v, bool inv, int &cache) {
  if (inv) v = -v;
  v = constrain(v, -255, 255);
  if (v == cache) return;          // evita reescrever o shift register à toa
  cache = v;
  if (v > 0)      { m.setSpeed(v);  m.run(FORWARD);  }
  else if (v < 0) { m.setSpeed(-v); m.run(BACKWARD); }
  else            { m.setSpeed(0);  m.run(RELEASE);  }
}

void mover(int esq, int dir) {
  setMotor(motorEsq, esq, INVERTE_ESQ, velAtualEsq);
  setMotor(motorDir, dir, INVERTE_DIR, velAtualDir);
}

// sentido > 0 = gira pra direita no próprio eixo | < 0 = esquerda
void girar(int sentido, int vel) {
  if (sentido > 0) mover(vel, -vel);
  else             mover(-vel, vel);
}

bool ehBranco(SensorIR &s, int leitura) {
  return s.brancoEhMenor ? (leitura < s.limiar) : (leitura > s.limiar);
}

long lerDistanciaCM() {
  digitalWrite(PINO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PINO_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_TRIG, LOW);
  unsigned long us = pulseIn(PINO_ECHO, HIGH, (unsigned long)DIST_MAX_CM * 58UL + 500UL);
  if (us == 0) return -1;          // nada no alcance
  return us / 58;
}

// ======================= CALIBRAÇÃO =======================

void loopCalibracao() {
  static unsigned long ultimo = 0;
  int v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = analogRead(ir[i].pino);
    if (v[i] < ir[i].minV) ir[i].minV = v[i];
    if (v[i] > ir[i].maxV) ir[i].maxV = v[i];
  }
  if (millis() - ultimo < 150) return;
  ultimo = millis();

  long d = lerDistanciaCM();

  for (int i = 0; i < 3; i++) {
    Serial.print(ir[i].nome);
    Serial.print(": ");
    Serial.print(v[i]);
    Serial.print(ehBranco(ir[i], v[i]) ? " BRANCO" : " preto ");
    Serial.print(" [min ");
    Serial.print(ir[i].minV);
    Serial.print(" max ");
    Serial.print(ir[i].maxV);
    Serial.print(" sug ");
    Serial.print((ir[i].minV + ir[i].maxV) / 2);
    Serial.print("]  |  ");
  }
  Serial.print("US: ");
  if (d < 0) Serial.println("---");
  else { Serial.print(d); Serial.println(" cm"); }
}

// ======================= LUTA =======================

enum Estado {
  BUSCA_1,        // gira 90° pra um lado
  BUSCA_2,        // volta 180° (fica 90° pro outro lado)
  BUSCA_GIRO,     // 360 contínuo
  BUSCA_AVANCA,   // anda um pouco e volta a girar
  ATAQUE,
  FUGA_RECUA,
  FUGA_GIRA,
  FUGA_FRENTE
};

#if DEBUG_ESTADOS
const char* NOMES[] = { "BUSCA_1", "BUSCA_2", "BUSCA_GIRO", "BUSCA_AVANCA",
                        "ATAQUE", "FUGA_RECUA", "FUGA_GIRA", "FUGA_FRENTE" };
#endif

Estado estado = BUSCA_1;
unsigned long tEstado = 0;
unsigned long ultimoVisto = 0;
unsigned long ultimoPing = 0;
long distancia = -1;
int sentidoGiro = 1;     // lado inicial da varredura
int sentidoFuga = 1;
int ladoFugaFrente = 0;  // -1 borda atrás-esq | 1 atrás-dir | 0 as duas

void entrar(Estado novo, unsigned long agora) {
#if DEBUG_ESTADOS
  if (novo != estado) Serial.println(NOMES[novo]);
#endif
  estado = novo;
  tEstado = agora;
}

void iniciarBusca(unsigned long agora) {
  sentidoGiro = -sentidoGiro;   // alterna o lado da varredura a cada busca
  entrar(BUSCA_1, agora);
}

void loopLuta() {
  unsigned long agora = millis();

  // --- Leitura das bordas ---
  int vF  = analogRead(ir[S_F].pino);
  int vBL = analogRead(ir[S_BL].pino);
  int vBR = analogRead(ir[S_BR].pino);
  bool bF  = ehBranco(ir[S_F],  vF);
  bool bBL = ehBranco(ir[S_BL], vBL);
  bool bBR = ehBranco(ir[S_BR], vBR);

  // --- Ultrassônico (com intervalo pra não pegar eco anterior) ---
  if (agora - ultimoPing >= INTERVALO_US_MS) {
    ultimoPing = agora;
    distancia = lerDistanciaCM();
  }
  bool oponente = (distancia > 0 && distancia <= DIST_ATAQUE_CM);

#if F_IR_DETECTA_OPONENTE
  oponente = oponente || bF;   // IR frontal vira gatilho de ataque
  bF = false;
#endif

  if (oponente) ultimoVisto = agora;

  // --- 1ª prioridade: não cair ---
  if (bF) {
    if (estado != FUGA_RECUA) sentidoFuga = -sentidoGiro;
    entrar(FUGA_RECUA, agora);             // renova o tempo enquanto ver branco
  }
  else if (bBL || bBR) {
    ladoFugaFrente = (bBL && bBR) ? 0 : (bBL ? -1 : 1);
    entrar(FUGA_FRENTE, agora);
  }
  else {
    unsigned long dt = agora - tEstado;
    switch (estado) {
      case FUGA_RECUA:
        if (dt >= TEMPO_RECUO) entrar(FUGA_GIRA, agora);
        break;

      case FUGA_GIRA:
        if (oponente) entrar(ATAQUE, agora);           // achou girando -> rush
        else if (dt >= TEMPO_GIRO_FUGA) iniciarBusca(agora);
        break;

      case FUGA_FRENTE:
        if (oponente) entrar(ATAQUE, agora);
        else if (dt >= TEMPO_FRENTE_FUGA) iniciarBusca(agora);
        break;

      case ATAQUE:
        if (!oponente && agora - ultimoVisto > TEMPO_PERDA) iniciarBusca(agora);
        break;

      case BUSCA_1:
        if (oponente) entrar(ATAQUE, agora);
        else if (dt >= TEMPO_90) entrar(BUSCA_2, agora);
        break;

      case BUSCA_2:
        if (oponente) entrar(ATAQUE, agora);
        else if (dt >= TEMPO_90 * 2) entrar(BUSCA_GIRO, agora);
        break;

      case BUSCA_GIRO:
        if (oponente) entrar(ATAQUE, agora);
        else if (dt >= TEMPO_GIRO_MAX) entrar(BUSCA_AVANCA, agora);
        break;

      case BUSCA_AVANCA:
        if (oponente) entrar(ATAQUE, agora);
        else if (dt >= TEMPO_AVANCA) entrar(BUSCA_GIRO, agora);
        break;
    }
  }

  // --- Saída nos motores ---
  switch (estado) {
    case ATAQUE:       mover(VEL_MAX, VEL_MAX);            break;
    case FUGA_RECUA:   mover(-VEL_MAX, -VEL_MAX);          break;
    case FUGA_GIRA:    girar(sentidoFuga, VEL_MAX);        break;
    case FUGA_FRENTE:
      if (ladoFugaFrente < 0)      mover(VEL_MAX, VEL_CURVA);  // borda atrás-esq -> puxa pra direita
      else if (ladoFugaFrente > 0) mover(VEL_CURVA, VEL_MAX);  // borda atrás-dir -> puxa pra esquerda
      else                         mover(VEL_MAX, VEL_MAX);
      break;
    case BUSCA_1:      girar(sentidoGiro,  VEL_BUSCA);     break;
    case BUSCA_2:      girar(-sentidoGiro, VEL_BUSCA);     break;
    case BUSCA_GIRO:   girar(-sentidoGiro, VEL_BUSCA);     break;
    case BUSCA_AVANCA: mover(VEL_AVANCA, VEL_AVANCA);      break;
  }
}

// ======================= SETUP / LOOP =======================

void setup() {
  Serial.begin(115200);
  pinMode(PINO_TRIG, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  digitalWrite(PINO_TRIG, LOW);
  mover(0, 0);

#if MODO_CALIBRACAO
  Serial.println(F("== CALIBRACAO == passe cada IR no preto e no branco"));
#else
  Serial.println(F("== LUTA == aguardando inicio"));
  for (int s = ESPERA_INICIO / 1000; s > 0; s--) {
    Serial.println(s);
    delay(1000);
  }
  Serial.println(F("VAI!"));
  tEstado = millis();
  sentidoGiro = -sentidoGiro;  // iniciarBusca vai alternar de volta pro lado inicial
  iniciarBusca(millis());
#endif
}

void loop() {
#if MODO_CALIBRACAO
  loopCalibracao();
#else
  loopLuta();
#endif
}
