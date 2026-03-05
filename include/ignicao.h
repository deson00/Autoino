static inline int normalizar_angulo_minimo_zero(int angulo) {
  while (angulo < 0) {
    angulo += 360;
  }
  if (angulo >= 360) {
    angulo = angulo % 360;
  }
  return angulo;
}

static inline byte indice_pino_ignicao(int i) {
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
    int fase_mod_120 = grau_pms % 120;
    if (fase_mod_120 < 0) {
      fase_mod_120 += 120;
    }

    if (fase_mod_120 <= 3) {
      ajuste_pms = 4;
    } else {
      ajuste_pms = 0;
    }
    return;
  }

  ajuste_pms = 0;
}

void calcula_grau_ignicao(int i){
if((captura_dwell[i] == false) && (ign_acionado[i] == false)){
      int angulo_base_ignicao = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
      angulo_base_ignicao = normalizar_angulo_minimo_zero(angulo_base_ignicao);

  if (angulo_base_ignicao == 0) {
    angulo_base_ignicao = 1;
  }

      unsigned long tempo_base_ignicao = (unsigned long)angulo_base_ignicao * tempo_cada_grau;
      unsigned long margem_minima = dwell_bobina + (tempo_cada_grau << 1);
      if (tempo_base_ignicao <= margem_minima) {
        tempo_base_ignicao = margem_minima + tempo_cada_grau;
      }
      tempo_proxima_ignicao[i] = tempo_base_ignicao;
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
      tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
  }
    if (i >= qtd_cilindro / 2){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false && referencia_posicao_sensor == true)){
      tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
  }
}

void iniciar_dwell_comando(int i){
    iniciar_dwell(i);
}

void desligar_dwell_comando(int i){
    desligar_dwell(i);
}