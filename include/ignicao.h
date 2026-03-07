static inline int normalizar_angulo_minimo_zero(int angulo) {
  while (angulo < 0) {
    angulo += 360;
  }
  if (angulo >= 360) {
    angulo = angulo % 360;
  }
  return angulo;
}

static inline unsigned long calcular_tempo_evento_ignicao(int angulo_alvo_graus) {
  if (tempo_cada_grau == 0) {
    return 1;
  }

  int angulo_normalizado = normalizar_angulo_minimo_zero(angulo_alvo_graus);

  // Se o ângulo é 0, o tempo deve ser 0 para disparar imediatamente no sincronismo.
  // Mapear 0 para 360 causava colisão fatal no ciclo de agendamento (BUG DOS 120 GRAUS).
  unsigned long tempo_evento_us = (unsigned long)angulo_normalizado * tempo_cada_grau;

  return tempo_evento_us;
}

static inline byte indice_pino_ignicao(int i) {
  // Se estiver lendo virabrequim (2 voltas), as saídas espelham-se pela metade!
  // No caso de 4 cilindros em Wasted Spark (centelha perdida no Vira), i=0 e i=2 vão para bobina A (indice 0);
  // i=1 e i=3 vão para bobina B (indice 1). Não passa do limite e não pisca a IGN3.
  if (local_rodafonica == 2 && modo_ignicao == 1) { // 1 = centelha perdida
    return (byte)(i % (qtd_cilindro / 2));
  }

  // Comportamento do fase/comando que era antigo (ignições emparelhadas)
  if (local_rodafonica == 1 && i >= (qtd_cilindro / 2)) {
    return (byte)(i - (qtd_cilindro / 2));
  }
  return (byte)i;
}

void atualizar_ajuste_pms_ignicao() {
  if (local_rodafonica == 1) {
    ajuste_pms = 0;
    return;
  }

  if (local_rodafonica == 2) {
    ajuste_pms = 0;
    return;
  }

  ajuste_pms = 0;
}

void calcula_grau_ignicao(int i){
if((captura_dwell[i] == false) && (ign_acionado[i] == false)){
      int angulo_base_ignicao = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
      angulo_base_ignicao = normalizar_angulo_minimo_zero(angulo_base_ignicao);
  tempo_proxima_ignicao[i] = calcular_tempo_evento_ignicao(angulo_base_ignicao);
    } 
}
void iniciar_dwell(int i){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) &&
        revolucoes_sincronizada >= 1 && status_corte == 0){ 
        byte pino = indice_pino_ignicao(i);
        captura_dwell[i] = true;
        tempo_percorrido[i] = micros();
        digitalWrite(ignicao_pins[pino], HIGH);
        // setPinHigh(ignicao_pins[i]);
        ign_acionado[i] = true;
    }
}

void desligar_dwell(int i){
      if ((captura_dwell[i] == true) && (ign_acionado[i] == true)) {
        byte pino = indice_pino_ignicao(i);
        captura_dwell[i] = false;
        ign_acionado[i] = false;
        digitalWrite(ignicao_pins[pino], LOW);
        // setPinLow(ignicao_pins[i]);
    
  }  
}

void calcula_dwell_comando(int i){
      if ( i < qtd_cilindro/2){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false && referencia_posicao_sensor == true)){
      int angulo_base_ignicao = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
      tempo_proxima_ignicao[i] = calcular_tempo_evento_ignicao(angulo_base_ignicao);
    }
  }
    if (i >= qtd_cilindro / 2){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false && referencia_posicao_sensor == true)){
      int angulo_base_ignicao = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
      tempo_proxima_ignicao[i] = calcular_tempo_evento_ignicao(angulo_base_ignicao);
    }
  }
}

void iniciar_dwell_comando(int i){
    iniciar_dwell(i);
}

void desligar_dwell_comando(int i){
    desligar_dwell(i);
}