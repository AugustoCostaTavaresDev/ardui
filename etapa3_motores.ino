// ETAPA 3 - Controle dos motores
// Esquerdo = M4, Direito = M2 (informado pelo usuario).
// Usa a biblioteca "Adafruit Motor Shield library" (V1, AFMotor.h).
// Nao usa os sensores. O mapa de pinos do shield so e considerado
// confirmado depois que o teste passar.
// Teste no chao: so com o caminho livre.

#include <AFMotor.h>

// ---------- Configuracao (ajustar durante o teste) ----------
const uint8_t  PWM_MAX     = 200;   // teto de PWM (0-255). Conservador ate medir a tensao no motor
const uint8_t  PASSO_MAX   = 8;     // variacao maxima de PWM por atualizacao
const uint16_t PERIODO_MS  = 20;    // 0 -> 200 em ~0,5 s
const uint16_t TIMEOUT_MS  = 5000;  // sem comando por este tempo: para sozinho
const bool     INVERTE_ESQ = false; // trocar para true se a roda esquerda girar ao contrario
const bool     INVERTE_DIR = false; // idem para a direita

// Mesma frequencia de PWM (~1 kHz) nos dois, pois M2 e M4 usam timers diferentes
AF_DCMotor mEsq(4, MOTOR34_1KHZ);   // esquerdo = M4
AF_DCMotor mDir(2, MOTOR12_1KHZ);   // direito  = M2

int8_t  sE = 0, sD = 0;             // sentido pedido: -1, 0, +1
uint8_t gE = 100, gD = 100;         // ganho por motor, % (so reduz: 100 = sem correcao)
uint8_t pct = 30;                   // velocidade, % do PWM_MAX
int16_t atualE = 0, atualD = 0;     // PWM atual (com sinal), depois da rampa
uint32_t tUltimoCmd = 0;

int16_t passoRampa(int16_t atual, int16_t alvo) {
  if (alvo > atual + PASSO_MAX) return atual + PASSO_MAX;
  if (alvo < atual - PASSO_MAX) return atual - PASSO_MAX;
  return alvo;
}

void aplica(AF_DCMotor &m, int16_t v, bool inverte) {
  if (v == 0) { m.setSpeed(0); m.run(RELEASE); return; }
  bool frente = (v > 0) != inverte;
  m.setSpeed(abs(v));
  m.run(frente ? FORWARD : BACKWARD);
}

void mostraGanho() {
  Serial.print(F("ganho esq=")); Serial.print(gE);
  Serial.print(F("%  dir="));    Serial.print(gD);
  Serial.println('%');
}

void ajuda() {
  Serial.println(F("1/2 esq frente/tras | 3/4 dir frente/tras | 5/6 ambos frente/tras"));
  Serial.println(F("7 gira p/ esquerda | 8 gira p/ direita | 0 para | + / - velocidade (5%)"));
  Serial.println(F("Ganho: a/q = esq -2/+2 | d/e = dir -2/+2 (50 a 100%)"));
}

void setup() {
  Serial.begin(115200);
  aplica(mEsq, 0, false);
  aplica(mDir, 0, false);
  ajuda();
  mostraGanho();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    bool ok = true;
    switch (c) {
      case '1': sE =  1; sD =  0; break;
      case '2': sE = -1; sD =  0; break;
      case '3': sE =  0; sD =  1; break;
      case '4': sE =  0; sD = -1; break;
      case '5': sE =  1; sD =  1; break;
      case '6': sE = -1; sD = -1; break;
      case '7': sE = -1; sD =  1; break;
      case '8': sE =  1; sD = -1; break;
      case '0': sE =  0; sD =  0; break;
      case '+': if (pct < 100) pct += 5; break;
      case '-': if (pct > 5)   pct -= 5; break;
      case 'a': if (gE > 50)  gE -= 2; mostraGanho(); break;
      case 'q': if (gE < 100) gE += 2; mostraGanho(); break;
      case 'd': if (gD > 50)  gD -= 2; mostraGanho(); break;
      case 'e': if (gD < 100) gD += 2; mostraGanho(); break;
      default:  ok = false;  // ignora \n, \r e outros
    }
    if (ok) tUltimoCmd = millis();
  }

  if (millis() - tUltimoCmd > TIMEOUT_MS) { sE = 0; sD = 0; }  // seguranca

  static uint32_t tAnterior = 0;
  if (millis() - tAnterior < PERIODO_MS) return;
  tAnterior = millis();

  int16_t base  = (int16_t)pct * PWM_MAX / 100;
  int16_t alvoE = (int16_t)((int32_t)base * gE / 100) * sE;
  int16_t alvoD = (int16_t)((int32_t)base * gD / 100) * sD;
  atualE = passoRampa(atualE, alvoE);
  atualD = passoRampa(atualD, alvoD);
  aplica(mEsq, atualE, INVERTE_ESQ);
  aplica(mDir, atualD, INVERTE_DIR);

  static int16_t ultE = 0x7FFF, ultD = 0x7FFF;
  static uint8_t ultPct = 0;
  if (atualE != ultE || atualD != ultD || pct != ultPct) {
    ultE = atualE; ultD = atualD; ultPct = pct;
    Serial.print(F("pct=")); Serial.print(pct);
    Serial.print(F("  PWM esq=")); Serial.print(atualE);
    Serial.print(F("  dir="));     Serial.println(atualD);
  }
}
