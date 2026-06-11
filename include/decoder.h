#define MIN_INTERVALO_DENTE_US 50        // Filtro anti-bounce mínimo
volatile uint32_t ultimo_tempo_interrupcao = 0;
volatile uint32_t intervalo_dente_referencia_us = 0;
volatile byte amostras_intervalo_validas = 0;

#define GAP_FATOR_MIN_FUNC_X10 15UL   // 1.5x em funcionamento
#define GAP_FATOR_MIN_PARTIDA_X10 14UL // 1.4x em partida para tolerar sweep/aceleracao
#define GAP_FATOR_MAX_MARGEM_X 4UL    // limite superior para rejeitar outliers
#define FATOR_RUIDO_DENTE_CURTO_NUM 3UL // Rejeita pulso menor que 1/3 do dente de referencia
#define FALHAS_SYNC_MAX_CONSECUTIVAS 3U

#define TEMPO_CADA_GRAU_MIN_US 1UL
#define TEMPO_CADA_GRAU_MAX_US 10000UL // Suporta partida lenta com rodas de poucos dentes (ex.: 4-1)
#define TEMPO_CADA_GRAU_ALPHA_DEN 4UL
#define TEMPO_CADA_GRAU_ALPHA_NUM 1UL

void agendar_eventos_motor_timer1();

static inline unsigned long limita_tempo_cada_grau(unsigned long valor_us) {
  if (valor_us < TEMPO_CADA_GRAU_MIN_US) {
    return TEMPO_CADA_GRAU_MIN_US;
  }
  if (valor_us > TEMPO_CADA_GRAU_MAX_US) {
    return TEMPO_CADA_GRAU_MAX_US;
  }
  return valor_us;
}

static inline unsigned long filtra_tempo_cada_grau(unsigned long tempo_instante_grau) {
  tempo_instante_grau = limita_tempo_cada_grau(tempo_instante_grau);
  if (tempo_cada_grau == 0) {
    return tempo_instante_grau;
  }
  // Filtro IIR inteiro: alpha = TEMPO_CADA_GRAU_ALPHA_NUM / TEMPO_CADA_GRAU_ALPHA_DEN.
  unsigned long tempo_filtrado = ((tempo_cada_grau * (TEMPO_CADA_GRAU_ALPHA_DEN - TEMPO_CADA_GRAU_ALPHA_NUM)) +
                                  (tempo_instante_grau * TEMPO_CADA_GRAU_ALPHA_NUM) +
                                  (TEMPO_CADA_GRAU_ALPHA_DEN >> 1)) /
                                 TEMPO_CADA_GRAU_ALPHA_DEN;
  return limita_tempo_cada_grau(tempo_filtrado);
}

void decoder_roda_fonica_padrao(){ //roda fonica padrao com quantidade de dente - dente faltante
  // tempo_inicial_codigo = micros(); // Registra o tempo inicial
  uint32_t tempo_agora = micros();
  if ((tempo_agora - ultimo_tempo_interrupcao) < MIN_INTERVALO_DENTE_US) {
    return; // Ignora o pulso, é ruído. Não faz nada, pois não é um pulso válido.
  }

  // Primeiro pulso valido: inicializa base de tempo e evita usar intervalo lixo.
  if (inicia_tempo_sensor_roda_fonica) {
    ultimo_tempo_interrupcao = tempo_agora;
    ultimo_pulso_rpm_us = tempo_agora;
    tempo_anterior = tempo_agora;
    tempo_atual = tempo_agora;
    intervalo_tempo_entre_dente = 0;
    tempo_dente_anterior[0] = 0;
    tempo_dente_anterior[1] = 0;
    intervalo_dente_referencia_us = 0;
    amostras_intervalo_validas = 0;
    leitura = 0;
    qtd_leitura = 0;
    falhas_sync_consecutivas = 0;
    verifica_falha = 0;
    inicia_tempo_sensor_roda_fonica = 0;
    return;
  }

  // Filtro adaptativo: rejeita dente espurio muito curto em relacao ao ultimo dente valido.
  unsigned long intervalo_candidato = (tempo_agora - tempo_anterior);
  if (intervalo_dente_referencia_us > 0 &&
      (intervalo_candidato * FATOR_RUIDO_DENTE_CURTO_NUM) < intervalo_dente_referencia_us) {
    return;
  }

  ultimo_tempo_interrupcao = tempo_agora;
  ultimo_pulso_rpm_us = tempo_agora;

  qtd_leitura++;
  tempo_atual = tempo_agora;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);

  unsigned long fator_gap_min_x10 = (rpm < rpm_partida) ? GAP_FATOR_MIN_PARTIDA_X10 : GAP_FATOR_MIN_FUNC_X10;
  unsigned long gap_minimo_us = 0;
  unsigned long gap_maximo_us = 0xFFFFFFFFUL;

  if (intervalo_dente_referencia_us > 0) {
    gap_minimo_us = (intervalo_dente_referencia_us * fator_gap_min_x10) / 10UL;
    gap_maximo_us = intervalo_dente_referencia_us * ((unsigned long)(qtd_dente_faltante + GAP_FATOR_MAX_MARGEM_X));
  }

  verifica_falha = gap_minimo_us;
  bool gap_detectado = (amostras_intervalo_validas >= 3) &&
                       (intervalo_dente_referencia_us > 0) &&
                       (intervalo_tempo_entre_dente > gap_minimo_us) &&
                       (intervalo_tempo_entre_dente < gap_maximo_us);

  // Mantem histórico simples para diagnostico/telemetria.
  tempo_dente_anterior[1] = tempo_dente_anterior[0];
  tempo_dente_anterior[0] = intervalo_tempo_entre_dente;
  //Serial.print("|");
  //Serial.print(qtd_leitura);

  // Re-arme automatico: se o gap deixar de ser reconhecido por muitos dentes seguidos,
  // limpa o estado para evitar ficar preso sem recuperar sincronismo.
  if (qtd_leitura > ((uint16_t)qtd_dente << 1)) {
    revolucoes_sincronizada = 0;
    qtd_leitura = 0;
    intervalo_dente_referencia_us = 0;
    amostras_intervalo_validas = 0;
  }

  if (gap_detectado)
    // if (verifica_falha < intervalo_tempo_entre_dente)
  {

    if (qtd_voltas == 1) {
      tempo_final_volta_completa = tempo_atual;
      tempo_total_volta_completa = (tempo_final_volta_completa - tempo_inicio_volta_completa);
      qtd_voltas = 0;
    }
    if (qtd_voltas == 0) {
      tempo_inicio_volta_completa = tempo_atual;
      qtd_voltas = 1;
    }
    //Serial.print("__");
    //Serial.println("");
    //Serial.print(posicao_atual_sensor); 
    qtd_revolucoes++;
    // Fallback por média da volta: usado apenas quando ainda não há atualização válida por dente.
    if (tempo_cada_grau == 0 && tempo_total_volta_completa > 0) {
      tempo_cada_grau = limita_tempo_cada_grau(tempo_total_volta_completa / 360UL);
    }

    // Mantem a posicao de fim de volta ate processar os cortes vencidos.
    // O evento final da centelha perdida no comando pode cair junto da falha.
    uint16_t dentes_esperados = (uint16_t)(qtd_dente - qtd_dente_faltante);
    // Em partida a velocidade do motor oscila bastante por compressao/bateria, entao aceita
    // uma janela maior de dentes para nao bloquear sincronismo inicial.
    uint16_t tolerancia_dentes = 1U;
    if (rpm < rpm_partida) {
      tolerancia_dentes = (uint16_t)qtd_dente_faltante + 3U;
      if (tolerancia_dentes < 3U) {
        tolerancia_dentes = 3U;
      }
    } else if (rpm < (rpm_partida + 800U)) {
      tolerancia_dentes = 2U;
    }
    bool contagem_valida = (qtd_leitura + tolerancia_dentes >= dentes_esperados) &&
                           (qtd_leitura <= dentes_esperados + tolerancia_dentes);

    if (!contagem_valida) {
      if (falhas_sync_consecutivas < 255) {
        falhas_sync_consecutivas++;
      }
      if (falhas_sync_consecutivas >= FALHAS_SYNC_MAX_CONSECUTIVAS) {
        revolucoes_sincronizada = 0;
      }
      if (++qtd_perda_sincronia >= 255) {
        qtd_perda_sincronia = 0;
      }
    } else {
      falhas_sync_consecutivas = 0;
      revolucoes_sincronizada++;
    }
    qtd_leitura = 0;
    intervalo_dente_referencia_us = 0;
    amostras_intervalo_validas = 0;
    if (tipo_ignicao_sequencial == 0 && revolucoes_sincronizada >= 1) {
      tempo_atual_proxima_ignicao[0] = tempo_atual;
      tempo_atual_proxima_injecao[0] = tempo_atual;
      agendar_eventos_motor_timer1();
    }

    posicao_atual_sensor = 0;

  } else {
    posicao_atual_sensor++;
    if (grau_cada_dente > 0) {
      unsigned long tempo_instante_grau = intervalo_tempo_entre_dente / grau_cada_dente;
      if (tempo_instante_grau > 0) {
        tempo_cada_grau = filtra_tempo_cada_grau(tempo_instante_grau);
      }
    }
    // enviar_byte_serial(grau_pms - (posicao_atual_sensor * grau_cada_dente), 1);
    if (rpm < rpm_limite_referencia_baixa_rotacao() && revolucoes_sincronizada >= 1) {
      if (local_rodafonica == 1 && tipo_ignicao_sequencial == 0) {
        ajuste_pms = 0;
        bool referencia_valida = false;
        for (int i = 0; i < qtd_cilindro; i++) {
          if ((captura_dwell[i] == false) && (ign_acionado[i] == false) &&
              angulo_referencia_ignicao_valido(i, grau_avanco)) {
            referencia_valida = true;
            break;
          }
        }
        referencia_posicao_sensor = referencia_valida; // mantém true se ao menos uma referência válida foi encontrada neste ciclo
      }
    }
    else {
      referencia_posicao_sensor = true; // Reseta a referência de posição do sensor quando o rpm estiver acima do rpm de partida, para permitir o ajuste normal do avanço
    }
    //enviar_byte_serial(tempo_cada_grau / 1000, 1);

  }
  // posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;
  intervalo_dente_referencia_us = intervalo_tempo_entre_dente;
  if (amostras_intervalo_validas < 255) {
    amostras_intervalo_validas++;
  }
  tempo_anterior = tempo_atual;
  // tempo_final_codigo = micros(); // Registra o tempo final
  // tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
}
void leitor_sensor_roda_fonica() {
//  tempo_inicial_codigo = micros(); // Registra o tempo inicial 
  // noInterrupts();
  //  TIMSK1 &= ~_BV(TOIE1);  // Desativa a interrupção do Timer1
  decoder_roda_fonica_padrao();
//  TIMSK1 |= _BV(TOIE1);   // Reativa a interrupção do Timer1
  // interrupts();
// tempo_final_codigo = micros(); // Registra o tempo final  
// tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;   
}
