#define MIN_INTERVALO_DENTE_US 50        // Filtro anti-bounce mínimo
volatile uint32_t ultimo_tempo_interrupcao = 0;
volatile uint32_t intervalo_dente_referencia_us = 0;
volatile byte amostras_intervalo_validas = 0;
volatile byte rejeicoes_dente_consecutivas = 0;
// DEBUG TEMPORARIO: quantas vezes o escape teve de liberar o filtro travado.
// Em regime saudavel deve ser 0; cada ocorrencia e uma morte permanente que
// foi evitada.
volatile uint16_t debug_escapes_filtro = 0;
// Depois de tantas rejeicoes seguidas do filtro de ruido, considera a
// referencia corrompida e a libera. Um surto de ruido real nunca dura tantos
// dentes seguidos; ja a referencia envenenada rejeita 100% deles, pra sempre.
#define REJEICOES_DENTE_MAX 6U

// Limiar minimo para considerar um intervalo como gap, RELATIVO ao tamanho
// real da falha. Com N dentes faltantes o gap dura (N+1) periodos de dente,
// entao um limiar fixo de 1.5x so e adequado para N=1 (gap de 2x). Numa
// roda 60-2 o gap e 3x, e exigir apenas 1.5x deixava o limiar la embaixo:
// bastava um dente atrasado em mais de meio periodo (ISR bloqueada) para
// virar gap falso no meio da volta. Colocando o limiar no meio do caminho
// entre 1x (dente normal) e (N+1)x (gap real) obtem-se a maior margem para
// os dois lados - e o dobro de tolerancia a atraso na roda 60-2.
//   N=1 -> 1.5x func / 1.4x partida (identico ao comportamento anterior)
//   N=2 -> 2.0x / 1.8x        N=3 -> 2.5x / 2.2x
#define GAP_FATOR_MIN_FUNC_X10(n) (10UL + ((unsigned long)(n) * 5UL))
#define GAP_FATOR_MIN_PARTIDA_X10(n) (10UL + ((unsigned long)(n) * 4UL))
#define GAP_FATOR_MAX_MARGEM_X 4UL    // limite superior para rejeitar outliers
#define FATOR_RUIDO_DENTE_CURTO_NUM 3UL // Rejeita pulso menor que 1/3 do dente de referencia
#define FALHAS_SYNC_MAX_CONSECUTIVAS 3U

#define TEMPO_CADA_GRAU_MIN_US 1UL
#define TEMPO_CADA_GRAU_MAX_US 10000UL // Suporta partida lenta com rodas de poucos dentes (ex.: 4-1)
#define TEMPO_CADA_GRAU_ALPHA_DEN 4UL
#define TEMPO_CADA_GRAU_ALPHA_NUM 1UL

// Reagendamento fino por dente: roda em TODO dente, mas so abaixo desse RPM.
// Em RPM baixo ha CPU de sobra pra recalcular sempre (mais precisao durante
// partida/marcha lenta); acima disso fica so com o calculo unico por volta
// (no dente de falha), pra nao competir por tempo com a interrupcao em RPM alto.
#define RECALCULO_AGENDAMENTO_RPM_MAXIMO 1000U

void agendar_eventos_motor_timer1();
void atualizar_agendamentos_ignicao_por_dente();
static inline uint32_t ler_tick32_timer1();

// TESTE: agendar_eventos_motor_timer1() e pesada (calcula todos os canais de
// ignicao/injecao) e so tem a janela de 1 dente pra rodar se ficar dentro da
// interrupcao. Ela nao depende de "posicao atual" (diferente do reagendamento
// por dente que ja tentamos antes) - o calculo dela e todo em cima de
// tick_base_sincronismo, capturado uma vez aqui. Por isso da pra so capturar
// esse tick na interrupcao (barato) e deixar o calculo pesado rodar no loop().
volatile bool agendamento_pendente = false;

// DEBUG TEMPORARIO: contador de falhas de contagem de dente. O detalhe
// (qtd/esperado/rpm) foi retirado depois que o decoder ficou 100% limpo
// (zero falhas em 4693 voltas); resta so o contador, para flagrar regressao.
volatile uint16_t debug_falhas_dente = 0;

// (contador de voltas perdidas removido: mediu 0 em todo o teste, hipotese
// descartada - o loop nunca deixou de processar uma volta)

// Referencia de "tamanho de dente normal": media filtrada dos dentes normais,
// e nao o intervalo do ultimo dente isolado. Todo o criterio de gap
// (gap_minimo = fator x referencia, gap_maximo = (N+4) x referencia) depende
// dela, entao um unico dente ruim contaminava a decisao: um dente atrasado
// encolhe o intervalo seguinte, a referencia desinflava e o gap REAL passava
// a nao caber mais na janela - assinatura FALHA_DENTE qtd=64..72 dos testes.
// IIR 3/4 antigo + 1/4 novo: segue a aceleracao em poucos dentes, mas um
// dente anomalo isolado mal desloca a referencia.
static inline void atualizar_referencia_dente(unsigned long intervalo_us) {
  if (intervalo_dente_referencia_us == 0) {
    intervalo_dente_referencia_us = intervalo_us;
    return;
  }
  // Trava de outlier: um dente atendido com atraso (CPU saturada) pode ser
  // medido como dezenas de ms. Sem isso ele envenena a referencia, que
  // alimenta tanto o filtro de ruido quanto a deteccao de gap. Acima de 2x a
  // referencia atual nao e dente normal - ou e o gap, ou e atraso de ISR.
  // Motor nenhum desacelera rapido o bastante pra dobrar o periodo do dente
  // de um dente pro outro, entao limitar aqui nao atrapalha nada legitimo.
  uint32_t limite = intervalo_dente_referencia_us << 1;
  if (intervalo_us > limite) {
    intervalo_us = limite;
  }
  intervalo_dente_referencia_us = intervalo_dente_referencia_us -
                                  (intervalo_dente_referencia_us >> 2) +
                                  (intervalo_us >> 2);
}

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
    // ESCAPE OBRIGATORIO. Este return acontece ANTES de qtd_leitura++ e ANTES
    // de atualizar_referencia_dente(). Consequencia: se a referencia for
    // inflada uma unica vez (um atraso grande de ISR medido como se fosse um
    // dente - visto em T=64644us no log), todo dente normal passa a ser
    // rejeitado, porque 176us*3 e muito menor que a referencia corrompida. E
    // ai o filtro tranca a porta por dentro: o re-arme automatico por
    // qtd_leitura > 2x qtd_dente nunca e alcancado (qtd_leitura nao incrementa
    // mais) e a referencia nao pode se autocorrigir (quem a atualiza esta
    // depois deste return). O estado vira permanente e insensivel ao RPM -
    // era esta a morte com o sinal do sensor perfeitamente limpo.
    if (rejeicoes_dente_consecutivas < 255) {
      rejeicoes_dente_consecutivas++;
    }
    if (rejeicoes_dente_consecutivas >= REJEICOES_DENTE_MAX) {
      intervalo_dente_referencia_us = 0; // libera o filtro e a deteccao de gap
      amostras_intervalo_validas = 0;    // evita gap falso na reconstrucao
      rejeicoes_dente_consecutivas = 0;
      if (debug_escapes_filtro < 65535) {
        debug_escapes_filtro++;
      }
    }
    return;
  }
  rejeicoes_dente_consecutivas = 0;

  ultimo_tempo_interrupcao = tempo_agora;
  ultimo_pulso_rpm_us = tempo_agora;

  qtd_leitura++;
  tempo_atual = tempo_agora;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);

  unsigned long fator_gap_min_x10 = (rpm < rpm_partida)
                                        ? GAP_FATOR_MIN_PARTIDA_X10(qtd_dente_faltante)
                                        : GAP_FATOR_MIN_FUNC_X10(qtd_dente_faltante);
  unsigned long gap_minimo_us = 0;
  unsigned long gap_maximo_us = 0xFFFFFFFFUL;

  if (intervalo_dente_referencia_us > 0) {
    gap_minimo_us = (intervalo_dente_referencia_us * fator_gap_min_x10) / 10UL;
    gap_maximo_us = intervalo_dente_referencia_us * ((unsigned long)(qtd_dente_faltante + GAP_FATOR_MAX_MARGEM_X));
  }

  verifica_falha = gap_minimo_us;
  uint16_t dentes_esperados = (uint16_t)(qtd_dente - qtd_dente_faltante);

  // Validacao de POSICAO do gap: o gap so pode aparecer no fim da volta.
  // Qualquer bloqueio de interrupcao (ISR do Timer1, secao critica no loop)
  // que atrase o atendimento de um dente faz o intervalo medido virar
  // interval+D; se D passar de meio periodo de dente, isso ultrapassa o
  // limiar de 1.5x e e confundido com o gap NO MEIO da volta - foi isso que
  // destruiu a contagem nos testes (FALHA_DENTE com qtd=3..43, muito longe
  // dos 58 esperados). O limiar de tempo cai junto com o periodo, entao o
  // problema piora com o RPM, exatamente como observado.
  // Ja sincronizado, so aceita gap em posicao plausivel; fora dela trata
  // como dente normal. Se o gap real chegar a ser rejeitado por engano, o
  // re-arme por qtd_leitura > 2x qtd_dente (acima) zera o sincronismo e
  // libera este filtro para reconquistar a referencia - sem travar.
  //
  // O limiar e PERCENTUAL da volta (nao um numero fixo de dentes), entao
  // acompanha rodas de qualquer tamanho. Aqui usa 75% via deslocamento
  // (x - x/4), que e barato o bastante para rodar em todo dente:
  //   60-2 -> 44 de 58     36-1 -> 27 de 35     12-1 -> 9 de 11
  // Para 80% basta trocar por (x - (x>>3) - (x>>4)), tambem so com shifts.
  uint16_t posicao_minima_gap = dentes_esperados - (dentes_esperados >> 2);
  bool posicao_gap_plausivel = (revolucoes_sincronizada < 1) ||
                               (qtd_leitura >= posicao_minima_gap);

  bool gap_detectado = (amostras_intervalo_validas >= 3) &&
                       (intervalo_dente_referencia_us > 0) &&
                       (intervalo_tempo_entre_dente > gap_minimo_us) &&
                       (intervalo_tempo_entre_dente < gap_maximo_us) &&
                       posicao_gap_plausivel;

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
    if (rpm >= rpm_partida && tempo_cada_grau == 0 && tempo_total_volta_completa > 0) {
      tempo_cada_grau = limita_tempo_cada_grau(tempo_total_volta_completa / 360UL);
    }

    // Mantem a posicao de fim de volta ate processar os cortes vencidos.
    // O evento final da centelha perdida no comando pode cair junto da falha.
    // (dentes_esperados ja calculado acima, para a validacao de posicao do gap)
    // Em partida a velocidade do motor oscila bastante por compressao/bateria, entao aceita
    // uma janela maior de dentes para nao bloquear sincronismo inicial.
    uint16_t tolerancia_dentes = 1U;
    if (rpm < rpm_partida) {
      tolerancia_dentes = 2U;
    } else if (rpm < (rpm_partida + 800U)) {
      tolerancia_dentes = 2U;
    }
    bool contagem_valida = (qtd_leitura + tolerancia_dentes >= dentes_esperados) &&
                           (qtd_leitura <= dentes_esperados + tolerancia_dentes);

    if (!contagem_valida) {
      if (debug_falhas_dente < 65535) {
        debug_falhas_dente++;
      }
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
    // A referencia NAO e mais zerada aqui: ela e uma media dos dentes normais
    // (ver atualizar_referencia_dente) e atravessa o gap ja valida, em vez de
    // ser reconstruida do zero - antes o primeiro valor pos-gap era o proprio
    // intervalo do gap, deixando a referencia inflada por varios dentes.
    amostras_intervalo_validas = 0;
    if (tipo_ignicao_sequencial == 0 && revolucoes_sincronizada >= 1) {
      tempo_atual_proxima_ignicao[0] = tempo_atual;
      tempo_atual_proxima_injecao[0] = tempo_atual;
      // So captura o tick de referencia aqui (barato) - o calculo pesado
      // (agendar_eventos_motor_timer1) roda depois, no loop().
      tick_base_sincronismo = ler_tick32_timer1();
      agendamento_pendente = true;
    }

    posicao_atual_sensor = 0;

  } else {
    posicao_atual_sensor++;
    if (grau_cada_dente > 0) {
      unsigned long tempo_instante_grau = intervalo_tempo_entre_dente / grau_cada_dente;
      if (tempo_instante_grau > 0) {
        tempo_cada_grau = filtra_tempo_cada_grau(tempo_instante_grau);
        // Reagendamento fino por dente: em todo dente, mas so em RPM baixo.
        if (rpm < RECALCULO_AGENDAMENTO_RPM_MAXIMO) {
          atualizar_agendamentos_ignicao_por_dente();
        }
      }
    }
    // enviar_byte_serial(grau_pms - (posicao_atual_sensor * grau_cada_dente), 1);
    //enviar_byte_serial(tempo_cada_grau / 1000, 1);

  }
  // posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;
  // So dente normal alimenta a referencia. O intervalo do gap vale (N+1)
  // periodos e envenenaria a media, justamente o que fazia a referencia ficar
  // errada nos primeiros dentes de cada volta.
  if (!gap_detectado) {
    atualizar_referencia_dente(intervalo_tempo_entre_dente);
    if (amostras_intervalo_validas < 255) {
      amostras_intervalo_validas++;
    }
  }
  tempo_anterior = tempo_atual;
  // tempo_final_codigo = micros(); // Registra o tempo final
  // tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
}
// A cronometragem da ISR (T=) foi retirada: ja mediu o que precisava
// (136us acima de 1000rpm, 256us em marcha lenta) e ela mesma custava ~8us
// em todo dente, alem da flash. Da pra recolocar quando formos otimizar a ISR.
void leitor_sensor_roda_fonica() {
  decoder_roda_fonica_padrao();
}
