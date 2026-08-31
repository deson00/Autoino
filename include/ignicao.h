static inline unsigned long calcular_tempo_evento_ignicao(int angulo_alvo_graus) {
  if (tempo_cada_grau == 0) {
    return 1;
  }

  int angulo_normalizado = normalizar_angulo_minimo_zero(angulo_alvo_graus);
  if (angulo_normalizado == 0 && angulo_alvo_graus > 0) {
    if (local_rodafonica == 2) {
      angulo_normalizado = MARGEM_IGNICAO_FIM_CICLO_GRAUS;
    } else {
      angulo_normalizado = 360 - MARGEM_IGNICAO_FIM_CICLO_GRAUS;
    }
  }

  // Em baixa rotacao e partida, atraso pequeno e mais seguro que adiantar antes do PMS.
  if (angulo_normalizado == 0) {
    return tempo_cada_grau;
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

static inline byte quantidade_eventos_ignicao_por_ciclo_sensor() {
  if (sensor_sem_falha()) {
    // Cada pulso do sensor ja e um evento de cilindro - nao ha N eventos
    // espalhados por uma volta, ha um evento por referencia.
    return 1;
  }
  if (modo_ignicao == 1 && local_rodafonica == 2) {
    return quantidade_canais_ignicao_fisicos();
  }
  return qtd_cilindro;
}

static inline byte indice_pino_ignicao(int i) {
  if (sensor_sem_falha()) {
    // Bobina unica: o distribuidor faz o roteamento mecanico, e no volante de
    // um dente (moto) ha uma bobina so, eventualmente dupla em paralelo para
    // gemeas - centelha perdida cobre os dois casos.
    return 0;
  }
  // Se estiver lendo virabrequim (2 voltas), as saídas espelham-se pela metade!
  // No caso de 4 cilindros em Wasted Spark (centelha perdida no Vira), i=0 e i=2 vão para bobina A (indice 0);
  // i=1 e i=3 vão para bobina B (indice 1). Não passa do limite e não pisca a IGN3.
  if (modo_ignicao == 1) { // 1 = centelha perdida
    byte canais = quantidade_canais_ignicao_fisicos();
    if (local_rodafonica == 1 && i >= canais) {
      return (byte)(i - canais);
    }
    return (byte)(i % canais);
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

static inline int calcular_angulo_ignicao_indice(int i) {
  int grau_pms_referencia = grau_pms;
  int grau_avanco_referencia = graus_avanco_para_referencia_sensor(grau_avanco);
  int separacao_eventos = grau_entre_cada_cilindro;
  if (modo_ignicao == 1 && local_rodafonica == 2) {
    separacao_eventos = 360 / quantidade_canais_ignicao_fisicos();
  }
  return ajuste_pms + grau_pms_referencia - offset_referencia_roda_fonica_graus() - grau_avanco_referencia + (separacao_eventos * i);
}

static inline unsigned long calcular_tempo_ignicao_indice(int i) {
  return calcular_tempo_evento_ignicao(calcular_angulo_ignicao_indice(i));
}

void calcula_grau_ignicao(int i){
if((captura_dwell[i] == false) && (ign_acionado[i] == false)){
  tempo_proxima_ignicao[i] = calcular_tempo_ignicao_indice(i);
    } 
}
void iniciar_dwell(int i){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) &&
        revolucoes_sincronizada >= 1 && status_corte == 0){ 
        byte pino = indice_pino_ignicao(i);
        captura_dwell[i] = true;
        tempo_percorrido[i] = micros();
        digitalWrite(ignicao_pins[pino], HIGH);
        ign_acionado[i] = true;
    }
}

// Guarda por ign_acionado, e ele e o PRIMEIRO a ser limpo.
//
// Antes exigia captura_dwell E ign_acionado, com ign_acionado limpo por ultimo.
// Entre as duas escritas havia uma janela de uma instrucao com ign_acionado
// verdadeiro e captura_dwell ja falso. Quem varre pedindo so ign_acionado -
// processar_cortes_vencidos e atualizar_compare_a_desligar - via "precisa
// desligar", chamava esta funcao, e ela nao fazia NADA porque captura_dwell ja
// era falso. A flag nao caia, a varredura reencontrava o mesmo canal, e o laco
// de replanejamento girava ate estourar as 32 tentativas.
//
// Janela de uma instrucao, o que explica a faixa estreita de rotacao em que o
// defeito aparecia: so pega quando a interrupcao cai exatamente ali.
//
// Limpar ign_acionado primeiro fecha a janela pelo outro lado: quem varre
// passa a ver "nao ha o que desligar", que e verdade, em vez de ver um pedido
// que ninguem atende.
void desligar_dwell(int i){
      if (ign_acionado[i] == true) {
        byte pino = indice_pino_ignicao(i);
        ign_acionado[i] = false;
        captura_dwell[i] = false;
        digitalWrite(ignicao_pins[pino], LOW);
        PULSO_ORFAO_BAIXO();
  }
}

// Protecao de dwell maximo: independente da causa (ainda em investigacao), garante
// que nenhuma bobina fique carregando por mais que MULTIPLICADOR_DWELL_MAX_X10/10
// vezes o dwell configurado. Margem relativa (nao um valor fixo em ms) para
// acompanhar automaticamente qualquer dwell_bobina configurado pela UI.
#define MULTIPLICADOR_DWELL_MAX_X10 15UL // 1.5x
volatile unsigned int contagem_protecao_dwell_maximo = 0;
void protege_dwell_maximo(){
  unsigned long limite_us = (dwell_bobina * MULTIPLICADOR_DWELL_MAX_X10) / 10UL;
  byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
  for (byte i = 0; i < eventos_ignicao; i++) {
    // Protegido contra corrida com as interrupcoes (mesmo padrao usado no resto
    // do agendamento): sem isso, uma interrupcao podia rearmar o canal para a
    // proxima centelha bem no meio desta checagem, e o "ignicao_agendada[i] = false"
    // logo em seguida apagava esse novo agendamento valido por engano.
    uint8_t sreg = SREG;
    cli();
    bool precisa_forcar = ign_acionado[i] && (micros() - tempo_percorrido[i] > limite_us);
#if DEBUG_PULSO_ISR_ALVO == 5
    // Diagnostico da bobina presa, codificado na LARGURA do pulso. Uma captura
    // responde as duas perguntas que faltam:
    //
    //   40us -> compare A DESARMADO. Ninguem ia desligar, e o caso e de arme
    //           perdido - problema em quem deveria armar.
    //   25us -> compare A armado, mas OCR1A NAO aponta para este canal. O
    //           timer esta esperando outro evento e este ficou orfao na fila.
    //   10us -> compare A armado E apontando para ca. O arme esta certo e a
    //           interrupcao simplesmente nao chegou a tempo - problema de
    //           latencia, nao de logica.
    uint8_t diag = 0;
    if (precisa_forcar) {
      if (!(TIMSK1 & (1 << OCIE1A))) diag = 1;
      else if (OCR1A != (uint16_t)ignicao_tick_desligar[i]) diag = 2;
      else diag = 3;
    }
#endif
    if (precisa_forcar) {
      desligar_dwell(i);
      ignicao_agendada[i] = false;
    }
    SREG = sreg;
#if DEBUG_PULSO_ISR_ALVO == 5
    if (diag) {
      PULSO_ALTO();
      if (diag == 1) _delay_us(40);
      else if (diag == 2) _delay_us(25);
      else _delay_us(10);
      PULSO_BAIXO();
    }
#endif
    if (precisa_forcar && contagem_protecao_dwell_maximo < 65535) {
      contagem_protecao_dwell_maximo++;
    }
  }
}

