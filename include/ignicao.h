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

  // Evita colar a faisca de 360 graus exatamente na borda do gap.
  if (angulo_normalizado == 0) {
    return 359UL * tempo_cada_grau;
  }

  unsigned long tempo_evento_us = (unsigned long)angulo_normalizado * tempo_cada_grau;

  return tempo_evento_us;
}

static inline byte quantidade_canais_ignicao_fisicos() {
  if (modo_ignicao == 1 && qtd_cilindro > 1) {
    byte canais = qtd_cilindro / 2;
    if (canais < 1) {
      canais = 1;
    }
    return canais;
  }
  return qtd_cilindro;
}

static inline byte indice_pino_ignicao(int i) {
  // Se estiver lendo virabrequim (2 voltas), as saídas espelham-se pela metade!
  // No caso de 4 cilindros em Wasted Spark (centelha perdida no Vira), i=0 e i=2 vão para bobina A (indice 0);
  // i=1 e i=3 vão para bobina B (indice 1). Não passa do limite e não pisca a IGN3.
  if (modo_ignicao == 1) { // 1 = centelha perdida
    return (byte)(i % quantidade_canais_ignicao_fisicos());
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
  int grau_pms_referencia = graus_virabrequim_para_referencia_sensor(grau_pms);
  int grau_avanco_referencia = graus_virabrequim_para_referencia_sensor(grau_avanco);
  int angulo_base_ignicao = ajuste_pms + grau_pms_referencia - grau_avanco_referencia + (grau_entre_cada_cilindro * i);
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
      int grau_pms_referencia = graus_virabrequim_para_referencia_sensor(grau_pms);
      int grau_avanco_referencia = graus_virabrequim_para_referencia_sensor(grau_avanco);
      int angulo_base_ignicao = ajuste_pms + grau_pms_referencia - grau_avanco_referencia + (grau_entre_cada_cilindro * i);
      tempo_proxima_ignicao[i] = calcular_tempo_evento_ignicao(angulo_base_ignicao);
    }
  }
    if (i >= qtd_cilindro / 2){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false && referencia_posicao_sensor == true)){
      int grau_pms_referencia = graus_virabrequim_para_referencia_sensor(grau_pms);
      int grau_avanco_referencia = graus_virabrequim_para_referencia_sensor(grau_avanco);
      int angulo_base_ignicao = ajuste_pms + grau_pms_referencia - grau_avanco_referencia + (grau_entre_cada_cilindro * i);
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
