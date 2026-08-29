static inline void desativar_pwm_marcha_lenta() {
#ifdef MARCHA_LENTA_PWM_TIMER2
  TCCR2A = 0;
  TCCR2B = 0;
#elif defined(MARCHA_LENTA_PWM_TIMER3)
  TCCR3A = 0;
  TCCR3B = 0;
#endif
  digitalWrite(pino_marcha_lenta, LOW);
}

static inline void aplicar_pwm_marcha_lenta(byte duty) {
#ifdef MARCHA_LENTA_PWM_TIMER2
  if (duty > 100) {
    duty = 100;
  }

  // Fast PWM de hardware no D11/OC2A: 16 MHz / (1024 * 256) = 61,0 Hz.
  OCR2A = (byte)(((unsigned int)duty * 255U) / 100U);
  TCCR2A = _BV(WGM20) | _BV(WGM21);
  if (duty > 0) {
    TCCR2A |= _BV(COM2A1);
  } else {
    digitalWrite(pino_marcha_lenta, LOW);
  }
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  saida_marcha_lenta_ativa = duty > 0;
#elif defined(MARCHA_LENTA_PWM_TIMER3)
  if (duty > 100) {
    duty = 100;
  }

  // Fast PWM de 8 bits no D5/OC3A do Mega: 16 MHz / (1024 * 256) = 61,0 Hz.
  OCR3A = (byte)(((unsigned int)duty * 255U) / 100U);
  TCCR3A = _BV(WGM30);
  if (duty > 0) {
    TCCR3A |= _BV(COM3A1);
  } else {
    digitalWrite(pino_marcha_lenta, LOW);
  }
  TCCR3B = _BV(WGM32) | _BV(CS32) | _BV(CS30);
  saida_marcha_lenta_ativa = duty > 0;
#else
  (void)duty;
  desativar_pwm_marcha_lenta();
  saida_marcha_lenta_ativa = false;
#endif
}

static inline byte calcular_pwm_marcha_lenta() {
  if (temperatura_desligamento_marcha_lenta == 0 ||
      temperatura_motor >= temperatura_desligamento_marcha_lenta) {
    return pwm_marcha_lenta_quente;
  }

  int diferenca = (int)pwm_marcha_lenta_quente - (int)pwm_marcha_lenta_frio;
  int duty = (int)pwm_marcha_lenta_frio +
             ((diferenca * (int)temperatura_motor) / (int)temperatura_desligamento_marcha_lenta);
  return (byte)constrain(duty, 0, 100);
}

static inline void corrigir_abertura_marcha_lenta(byte abertura_base) {
  const byte TPS_MAXIMO_CONTROLE = 5;
  const byte HISTERESE_RPM = 50;

  if (rpm <= rpm_partida || valor_tps > TPS_MAXIMO_CONTROLE) {
    abertura_marcha_lenta_atual = abertura_base;
    return;
  }

  if (rpm + HISTERESE_RPM < rpm_alvo_marcha_lenta) {
    if (abertura_marcha_lenta_atual < 100) {
      abertura_marcha_lenta_atual++;
    }
  } else if (rpm > rpm_alvo_marcha_lenta + HISTERESE_RPM) {
    if (abertura_marcha_lenta_atual > 0) {
      abertura_marcha_lenta_atual--;
    }
  }
}

static inline void definir_direcao_passo_marcha_lenta(bool abrir) {
  bool nivel_alto = abrir != (inverter_direcao_marcha_lenta != 0);
  digitalWrite(pino_direcao_marcha_lenta, nivel_alto ? HIGH : LOW);
}

static inline void pulsar_passo_marcha_lenta() {
  digitalWrite(pino_passo_marcha_lenta, HIGH);
  delayMicroseconds(2);
  digitalWrite(pino_passo_marcha_lenta, LOW);
}

void processar_motor_passo_marcha_lenta() {
  if (modo_marcha_lenta != 3 || posicao_passo_marcha_lenta == alvo_passo_marcha_lenta) {
    return;
  }

  unsigned long agora_us = micros();
  if ((agora_us - ultimo_passo_marcha_lenta_us) < 1000UL) {
    return;
  }
  ultimo_passo_marcha_lenta_us = agora_us;

  bool abrir = posicao_passo_marcha_lenta < alvo_passo_marcha_lenta;
  definir_direcao_passo_marcha_lenta(abrir);
  pulsar_passo_marcha_lenta();
  if (abrir) {
    posicao_passo_marcha_lenta++;
  } else {
    posicao_passo_marcha_lenta--;
  }
}

static void referenciar_motor_passo_marcha_lenta() {
  desativar_pwm_marcha_lenta();
  definir_direcao_passo_marcha_lenta(false);
  unsigned int passos_homing = maximo_passos_marcha_lenta + 10U;
  for (unsigned int i = 0; i < passos_homing; i++) {
    pulsar_passo_marcha_lenta();
    delayMicroseconds(1000);
  }
  posicao_passo_marcha_lenta = 0;
  alvo_passo_marcha_lenta = 0;
  ultimo_passo_marcha_lenta_us = micros();
}

void atualizar_controle_marcha_lenta() {
  if (modo_marcha_lenta == 0) {
    desativar_pwm_marcha_lenta();
    saida_marcha_lenta_ativa = false;
    return;
  }

  if (modo_marcha_lenta == 2) {
    corrigir_abertura_marcha_lenta(calcular_pwm_marcha_lenta());
    aplicar_pwm_marcha_lenta(abertura_marcha_lenta_atual);
    return;
  }

  if (modo_marcha_lenta == 3) {
    corrigir_abertura_marcha_lenta(calcular_pwm_marcha_lenta());
    alvo_passo_marcha_lenta = ((unsigned int)maximo_passos_marcha_lenta *
                               abertura_marcha_lenta_atual) / 100U;
    saida_marcha_lenta_ativa = alvo_passo_marcha_lenta > 0;
    return;
  }

#ifdef MARCHA_LENTA_PWM_TIMER2
  TCCR2A = 0;
  TCCR2B = 0;
#elif defined(MARCHA_LENTA_PWM_TIMER3)
  TCCR3A = 0;
  TCCR3B = 0;
#endif
  if (temperatura_desligamento_marcha_lenta == 0) {
    saida_marcha_lenta_ativa = false;
    digitalWrite(pino_marcha_lenta, LOW);
    return;
  }

  byte temperatura_rearme = temperatura_desligamento_marcha_lenta > histerese_marcha_lenta
                              ? temperatura_desligamento_marcha_lenta - histerese_marcha_lenta
                              : 0;

  if (saida_marcha_lenta_ativa) {
    if (temperatura_motor >= temperatura_desligamento_marcha_lenta) {
      saida_marcha_lenta_ativa = false;
    }
  } else if (temperatura_motor <= temperatura_rearme) {
    saida_marcha_lenta_ativa = true;
  }

  digitalWrite(pino_marcha_lenta, saida_marcha_lenta_ativa ? HIGH : LOW);
}

void inicializar_controle_marcha_lenta() {
  pinMode(pino_marcha_lenta, OUTPUT);
  pinMode(pino_passo_marcha_lenta, OUTPUT);
  pinMode(pino_direcao_marcha_lenta, OUTPUT);
  desativar_pwm_marcha_lenta();
  saida_marcha_lenta_ativa = false;
  abertura_marcha_lenta_atual = calcular_pwm_marcha_lenta();
  if (modo_marcha_lenta == 3) {
    referenciar_motor_passo_marcha_lenta();
  }
  atualizar_controle_marcha_lenta();
}
