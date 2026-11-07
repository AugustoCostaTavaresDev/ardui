#include <AFMotor.h>

enum Diag { D_DESCONHECIDO, D_OK, D_BORDA_ESQ, D_BORDA_DIR, D_FRONTAL,
            D_FRONTAL_ESQ, D_FRONTAL_DIR, D_INCONSISTENTE, D_FORA };

const uint8_t PIN_E = A4;
const uint8_t PIN_C = A0;   
const uint8_t PIN_D = A5;
const uint8_t PIN_START = A3; 
const uint8_t PINOS[3] = { PIN_E, PIN_C, PIN_D };
const uint8_t LED_PLACA = 13;

const uint8_t PIN_TRIG = A2;
const uint8_t PIN_ECHO = A1;

const int LIMIAR_MAO = 600; 
const int DIST_MINIMA = 15;

const float POS_X[3] = { -19.2, 0.0, 19.2 };
const float X_REF    = 19.2;
float lado[3];

const int16_t  W_BRANCO   = 650;
const int16_t  W_PRETO    = 350;
const uint16_t SPAN_MIN   = 60;
const uint8_t  N_AMOSTRAS = 8;

uint16_t branco[3] = { 32, 28, 52 };
uint16_t preto[3]  = { 340, 317, 239 };
bool temBranco = true; 
bool temPreto = true;

float kp = 40.0f;                      
float kd = 1.5f;                       
float V_RETA = 100.0f;                 
float V_CURVA = 100.0f;                
const float V_RAMPA = 120.0f;          
const float ALFA_D = 0.3f;             
const uint32_t PERIODO_US = 10000;     
const uint16_t T_ARMA_MS = 1000;       
const uint8_t  N_PARADA = 3;           

const uint8_t PWM_MAX     = 200;       
const bool    INVERTE_ESQ = false;
const bool    INVERTE_DIR = false;
int OFFSET_ESQ = 30; 
int OFFSET_DIR = 30;

// ---- recuperacao: giro no proprio eixo ----
// #define TESTE_GIRO   // descomente para medir T_GIRO_90_MS: gira no eixo a cada 3 s, sem sensores
const float    V_GIRO         = 60.0f;  // giro no eixo: uma roda +V_GIRO, a outra -V_GIRO
uint16_t       T_GIRO_90_MS   = 250;    // tempo para ~90 graus no eixo. MEDIR no chao e ajustar
int            SENTIDO_PADRAO = 1;      // ultimo desempate do lado (sem nenhuma pista): 1 = esquerda, -1 = direita
const float    FRAC_GIRO_MIN  = 0.5f;   // tMin = FRAC_GIRO_MIN * T_GIRO_90_MS: giro minimo (sai da parede) antes de olhar o sensor
const float    FRAC_GIRO_MAX  = 1.0f;   // tMax = FRAC_GIRO_MAX * T_GIRO_90_MS (~90 graus): sem saida ate aqui -> PARADO (nunca vira de volta)
const uint16_t JANELA_TRECHO_MS = 300;  // FRONTAL ate este tempo apos o fim de uma recuperacao = MESMO trecho (re-bateu na mesma parede); depois = trecho NOVO -> vira p/ o lado oposto
const uint8_t  N_OK_GIRO      = 3;      // ciclos (10 ms) seguidos com d == D_OK para encerrar o giro
const float    LIM_EANT       = 0.05f;  // zona morta de eAnt na escolha do lado (abaixo disso = sem informacao)
const uint16_t T_RE_MS        = 500;    // duracao da re antes do giro (0 = sem re; testar 150-200 ms)
const uint8_t  PWM_PIVO_MIN   = 180;    // piso de PWM no pivo, p/ bateria baixa (0 = desliga o piso). Revalidar T_GIRO_90_MS ao mudar

AF_DCMotor mEsq(4, MOTOR34_1KHZ);
AF_DCMotor mDir(2, MOTOR12_1KHZ);

uint8_t estado[3] = { 255, 255, 255 }; 

float lerDistancia() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duracao = pulseIn(PIN_ECHO, HIGH, 3000);
  if (duracao == 0) return 999;
  return duracao * 0.034 / 2;
}

uint16_t leSensor(uint8_t pino) {
  analogRead(pino);   
  uint32_t soma = 0;
  for (uint8_t i = 0; i < N_AMOSTRAS; i++) soma += analogRead(pino);
  return soma / N_AMOSTRAS;
}

int16_t brancura(uint8_t i, uint16_t v) {
  if (!(temBranco && temPreto)) return -1;
  int32_t span = (int32_t)preto[i] - (int32_t)branco[i];
  if (span < (int32_t)SPAN_MIN) return -1;
  int32_t n = ((int32_t)v - (int32_t)branco[i]) * 1000L / span;   
  if (n < 0) n = 0;
  if (n > 1000) n = 1000;
  return (int16_t)(1000 - n);
}

void atualizaEstado(uint8_t i, int16_t w) {
  if (w < 0) { estado[i] = 255; return; }
  if (w >= W_BRANCO)     estado[i] = 0;
  else if (w <= W_PRETO) estado[i] = 1;
}

Diag classifica() {
  for (uint8_t i = 0; i < 3; i++) if (estado[i] == 255) return D_DESCONHECIDO;
  uint8_t m = (estado[0] == 0 ? 1 : 0) | (estado[1] == 0 ? 2 : 0) | (estado[2] == 0 ? 4 : 0);
  switch (m) {
    case 0: return D_OK;
    case 1: return D_BORDA_ESQ;
    case 4: return D_BORDA_DIR;
    case 2: return D_FRONTAL;
    case 3: return D_FRONTAL_ESQ;
    case 6: return D_FRONTAL_DIR;
    case 5: return D_INCONSISTENTE;
    default: return D_FORA;
  }
}

void setMotor(AF_DCMotor &m, float v, bool inverte, int offset, int piso) {
  if (v > 100.0f) v = 100.0f;
  if (v < -100.0f) v = -100.0f;
  if (v == 0.0f) { m.setSpeed(0); m.run(RELEASE); return; }
  
  float v_abs = (v > 0) ? v : -v;
  int pwm_final = (int)(v_abs * PWM_MAX / 100.0f) + offset;
  if (pwm_final < piso) pwm_final = piso;         // piso (so o pivo usa; demais chamadas passam 0)
  if (pwm_final > 255) pwm_final = 255;
  if (pwm_final < 0) pwm_final = 0;

  m.setSpeed((uint8_t)pwm_final);
  bool frente = (v > 0);
  if (inverte) frente = !frente;
  m.run(frente ? FORWARD : BACKWARD);
}

void setMotores(float vE, float vD) {
  setMotor(mEsq, vE, INVERTE_ESQ, OFFSET_ESQ, 0);
  setMotor(mDir, vD, INVERTE_DIR, OFFSET_DIR, 0);
}

// pivo no eixo: dir = 1 gira p/ esquerda, -1 p/ direita. Usa PWM_PIVO_MIN como piso
void pivota(int dir) {
  float v = (dir == 1) ? V_GIRO : -V_GIRO;
  setMotor(mEsq, -v, INVERTE_ESQ, OFFSET_ESQ, PWM_PIVO_MIN);
  setMotor(mDir,  v, INVERTE_DIR, OFFSET_DIR, PWM_PIVO_MIN);
}

enum Ctl { PARADO, ARMADO, RODANDO, RE_DANDO, PAUSA_RE, GIRANDO_BUSCA };
Ctl ctl = PARADO;
uint32_t tArma = 0, tInicio = 0, t_recuperacao = 0;
float vb = 0, eAnt = 0, dFilt = 0;
uint8_t contParada = 0;
bool ultValido = false;
Diag ultDiag = D_DESCONHECIDO;
uint32_t t_sem_borda = 0;
float vAlvo = V_CURVA;

int direcao_giro = 1;
int ultima_dir = 0;            // sentido da ultima recuperacao nesta corrida (0 = ainda nao houve)
uint8_t contOkGiro = 0;        // ciclos seguidos com d == D_OK durante o giro
uint32_t giro_ms = 0;          // tempo de giro (so para o log)
float eMem = 0;                // memoria do lado da borda (decai em ~0,5 s)
uint32_t t_fim_rec = 0;

// ---- LOG de diagnostico (comente a linha abaixo para desligar) ----
#define LOG_EVENTOS
#ifdef LOG_EVENTOS
// imprime: tempo | evento | diagnostico | brancura E,C,D | eAnt | eMem | sentido | tempo de giro (ms)
void logEv(const char* ev, Diag d, int16_t* w) {
  Serial.print(millis()); Serial.print(F(" | ")); Serial.print(ev);
  Serial.print(F(" | d=")); Serial.print((int)d);
  Serial.print(F(" w=")); Serial.print(w[0]); Serial.print(','); Serial.print(w[1]); Serial.print(','); Serial.print(w[2]);
  Serial.print(F(" eAnt=")); Serial.print(eAnt, 2);
  Serial.print(F(" eMem=")); Serial.print(eMem, 2);
  Serial.print(F(" dir=")); Serial.print(direcao_giro);
  Serial.print(F(" tg=")); Serial.println(giro_ms);
}
#else
#define logEv(a, b, c)
#endif 

void setup() {
  Serial.begin(115200);
  pinMode(LED_PLACA, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  
  for (uint8_t i = 0; i < 3; i++) lado[i] = POS_X[i] / X_REF;
  setMotores(0, 0);
  
  for(int i=0; i<3; i++){
    digitalWrite(LED_PLACA, HIGH); delay(200);
    digitalWrite(LED_PLACA, LOW); delay(200);
  }
}

void loop() {
#ifdef TESTE_GIRO
  delay(3000); pivota(1); delay(T_GIRO_90_MS); setMotores(0, 0); return;
#endif
  int leitura_mao = analogRead(PIN_START);
  bool mao_presente = (leitura_mao > LIMIAR_MAO); 

  static uint32_t tAnt = 0;
  uint32_t agora = micros();
  if (agora - tAnt < PERIODO_US) return;
  float dt = (agora - tAnt) * 1e-6f;
  if (dt > 0.05f) dt = 0.05f;
  tAnt = agora;

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
  if (valido) for (uint8_t i = 0; i < 3; i++) erro += lado[i] * w[i];
  
  if (d == D_FRONTAL || d == D_FRONTAL_ESQ || d == D_FRONTAL_DIR) {
      if (eAnt > 0) erro = 1000.0f;   
      else if (eAnt < 0) erro = -1000.0f; 
      else erro = 1000.0f; 
  }

  float e = erro / 1000.0f;
  ultValido = valido;
  static Diag dLog = D_DESCONHECIDO;
  if (d != dLog) { logEv("diag muda", d, w); dLog = d; }
  ultDiag = d;

  if (ctl == PARADO && mao_presente) {
      if (ultValido && ultDiag == D_OK) {
          ctl = ARMADO;
          tArma = millis();
          digitalWrite(LED_PLACA, HIGH); 
      } else {
          digitalWrite(LED_PLACA, !digitalRead(LED_PLACA));
          delay(50);
      }
  }

  if (ctl == ARMADO) {
    if (!mao_presente && (millis() - tArma >= T_ARMA_MS)) {
      ctl = RODANDO;
      tInicio = millis();
      digitalWrite(LED_PLACA, LOW); 
      vb = 0; eAnt = e; dFilt = 0; contParada = 0;
      t_sem_borda = millis();
      ultima_dir = 0;
    }
  }

  if (ctl == RE_DANDO) {
      setMotores(-50, -50); 
      if (millis() - t_recuperacao > T_RE_MS) {
          ctl = PAUSA_RE;
          t_recuperacao = millis();
      }
      return; 
  }

  if (ctl == PAUSA_RE) {
      setMotores(0, 0); 
      if (millis() - t_recuperacao > 100) {
          ctl = GIRANDO_BUSCA;
          t_recuperacao = millis();
          contOkGiro = 0;
      }
      return;
  }

  if (ctl == GIRANDO_BUSCA) {
      uint32_t tg = millis() - t_recuperacao;
      uint32_t tMin = (uint32_t)(FRAC_GIRO_MIN * T_GIRO_90_MS);
      uint32_t tMax = (uint32_t)(FRAC_GIRO_MAX * T_GIRO_90_MS);
      giro_ms = tg;
      contOkGiro = (tg >= tMin && d == D_OK) ? contOkGiro + 1 : 0;   // so conta depois do tMin
      if (contOkGiro >= N_OK_GIRO) {                           // saiu da parede e a frente esta livre
          logEv("giro OK->roda", d, w);
          setMotores(0, 0);
          ctl = RODANDO;
          vb = 0; eAnt = 0; eMem = 0; dFilt = 0; contParada = 0;
          t_sem_borda = millis(); t_fim_rec = millis();
      } else if (tg >= tMax) {                                 // sem saida ate ~90 graus: conservador (nunca vira de volta)
          logEv("giro TIMEOUT->parado", d, w);
          setMotores(0, 0);
          ctl = PARADO;
          digitalWrite(LED_PLACA, HIGH);
      } else {
          pivota(direcao_giro);                                // no eixo, sempre no sentido escolhido
      }
      return;
  }

  if (ctl == RODANDO) {
    float distancia = lerDistancia();
    
    if (d == D_FRONTAL || d == D_FRONTAL_ESQ || d == D_FRONTAL_DIR || (distancia < DIST_MINIMA)) {
        ctl = RE_DANDO;
        t_recuperacao = millis();
        
        // Lado do giro, em ordem de confianca (nao inverte mais por "repeticao": numa pista curta isso gera ping-pong):
        //  1) diagnostico: borda a frente-esquerda -> gira p/ direita; frente-direita -> gira p/ esquerda
        //  2) so o central branco: o lateral mais branco indica a borda mais proxima -> gira para o lado oposto
        //  3) TRECHO NOVO (ja houve recuperacao e passou JANELA_TRECHO_MS): a pista alterna -> lado OPOSTO ao da ultima
        //  4) sinal de eAnt (lado da borda no ultimo ciclo): e > 0 = borda a direita -> gira p/ esquerda
        //  5) memoria do lado da borda (eMem)
        //  6) mesmo trecho sem pista nenhuma: repete o lado da ultima (continua saindo da mesma parede)
        //  7) SENTIDO_PADRAO = so na 1a recuperacao da corrida, sem pista nenhuma
        int dir;
        const char* crit;
        int dw = w[2] - w[0];                       // > 0: mais branco a direita
        bool tem = (ultima_dir != 0);
        bool mesmo = tem && (millis() - t_fim_rec < JANELA_TRECHO_MS);
        if (d == D_FRONTAL_ESQ)                { dir = -1; crit = "FRONTAL->re lado=diag"; }
        else if (d == D_FRONTAL_DIR)           { dir = 1;  crit = "FRONTAL->re lado=diag"; }
        else if (valido && dw > 150)           { dir = 1;  crit = "FRONTAL->re lado=dw"; }
        else if (valido && dw < -150)          { dir = -1; crit = "FRONTAL->re lado=dw"; }
        else if (tem && !mesmo)         { dir = -ultima_dir; crit = "FRONTAL->re lado=alterna"; }
        else if (eAnt > LIM_EANT)       { dir = 1;  crit = "FRONTAL->re lado=eAnt"; }
        else if (eAnt < -LIM_EANT)             { dir = -1; crit = "FRONTAL->re lado=eAnt"; }
        else if (eMem > 0.08f)                 { dir = 1;  crit = "FRONTAL->re lado=eMem"; }
        else if (eMem < -0.08f)                { dir = -1; crit = "FRONTAL->re lado=eMem"; }
        else if (tem)                          { dir = ultima_dir; crit = "FRONTAL->re lado=mesmo"; }
        else                                   { dir = SENTIDO_PADRAO; crit = "FRONTAL->re lado=padrao"; }
        direcao_giro = dir; ultima_dir = dir; giro_ms = 0;
        logEv(crit, d, w);
        
        return;
    }

    bool parada = !valido || d == D_DESCONHECIDO || d == D_FORA;
    contParada = parada ? contParada + 1 : 0;

    if (contParada >= N_PARADA) {
      setMotores(0, 0);
      ctl = PARADO;
      digitalWrite(LED_PLACA, HIGH); 
    } else {
      if (w[0] > 200 || w[2] > 200) t_sem_borda = millis();
      
      if (millis() - t_sem_borda > 400) vAlvo = V_RETA;
      else vAlvo = V_CURVA;

      float de = (e - eAnt) / dt;
      eAnt = e;
      eMem += 0.02f * (e - eMem);
      dFilt += ALFA_D * (de - dFilt);
      float u = kp * e + kd * dFilt;
      
      if (vb < vAlvo) {
          vb += V_RAMPA * dt;
          if (vb > vAlvo) vb = vAlvo;
      } else if (vb > vAlvo) {
          vb -= V_RAMPA * 2.0f * dt; 
          if (vb < vAlvo) vb = vAlvo;
      }

      float vE = vb - u, vD = vb + u;
      // saturacao: preserva a DIFERENCA entre as rodas (a direcao) e sacrifica a velocidade media
      float exc = vD - 100.0f;  if (exc > 0) { vD -= exc; vE -= exc; }
      exc = vE - 100.0f;        if (exc > 0) { vE -= exc; vD -= exc; }
      exc = -100.0f - vE;       if (exc > 0) { vE += exc; vD += exc; }
      exc = -100.0f - vD;       if (exc > 0) { vD += exc; vE += exc; }
      if (vE < -100.0f) vE = -100.0f;
      if (vE > 100.0f) vE = 100.0f;
      if (vD < -100.0f) vD = -100.0f;
      if (vD > 100.0f) vD = 100.0f;
      
      setMotores(vE, vD);
    }
  }
}