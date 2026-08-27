#define MIN_INTERVALO_DENTE_US 50        // Filtro anti-bounce mínimo
volatile uint32_t ultimo_tempo_interrupcao = 0;
volatile uint32_t intervalo_dente_referencia_us = 0;
volatile byte amostras_intervalo_validas = 0;
volatile byte rejeicoes_dente_consecutivas = 0;
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
// Expresso como numerador sobre denominador 8, para o limiar sair por
// deslocamento em vez de divisao: limiar = referencia * NUM >> 3. A divisao
// de 32 bits por 10 que havia aqui custava ~300 ciclos dentro da interrupcao
// do dente, que e o gargalo do RPM maximo.
//   funcionamento: (4N+8)/8 = 1 + N/2  -> N=1:1.5x   N=2:2.0x   N=3:2.5x
//   partida:       (3N+8)/8            -> N=1:1.375x N=2:1.75x  N=3:2.125x
// Os valores de funcionamento sao exatamente os mesmos de antes; os de
// partida ficam um pouco mais permissivos (1.4->1.375, 1.8->1.75), o que e o
// lado seguro: durante a partida a rotacao oscila e errar detectando o gap e
// pior do que aceita-lo com folga.
#define GAP_FATOR_MIN_FUNC_NUM(n) (8UL + ((unsigned long)(n) * 4UL))
#define GAP_FATOR_MIN_PARTIDA_NUM(n) (8UL + ((unsigned long)(n) * 3UL))
#define GAP_FATOR_MIN_SHIFT 3
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

// Divisao por grau_cada_dente sem divisao de 32 bits. Ela roda em TODO dente
// e custava ~300 ciclos dentro da interrupcao, que e o gargalo do RPM maximo.
// grau_cada_dente vem da configuracao e praticamente nunca muda, entao o
// reciproco fica em cache e so e recalculado quando o valor muda - no caminho
// quente sobra uma comparacao de byte e uma multiplicacao 32x16.
// Precisao (verificada contra a divisao inteira em toda a faixa util, para
// grau de 1 a 360): reciproco arredondado em Q18 com resultado truncado da
// resultado IDENTICO a divisao para as rodas usuais - inclusive grau=6 da
// 60-2. So rodas de 1 a 2 dentes (grau 180/360) chegam a divergir 3us, e
// mesmo esse valor ainda passa pelo filtro IIR de tempo_cada_grau. Q18 e o
// maior expoente seguro: o produto maximo e 10000*2^18 = 2,6e9, dentro de
// 32 bits para qualquer grau, porque a entrada ja e limitada logo abaixo.
#define GRAU_RECIPROCO_SHIFT 18
static inline unsigned long dividir_por_grau_cada_dente(unsigned long intervalo_us) {
  static byte grau_em_cache = 0;
  static uint32_t reciproco = 0;
  static uint32_t limite_entrada_us = 0;

  if (grau_em_cache != grau_cada_dente) {
    grau_em_cache = grau_cada_dente;
    reciproco = ((1UL << GRAU_RECIPROCO_SHIFT) + (grau_cada_dente >> 1)) / grau_cada_dente;
    limite_entrada_us = TEMPO_CADA_GRAU_MAX_US * (unsigned long)grau_cada_dente;
  }

  // Teto de entrada, por dois motivos que coincidem: acima dele o resultado
  // seria saturado por limita_tempo_cada_grau de qualquer jeito, e a
  // multiplicacao estouraria 32 bits (em rotacao muito baixa o intervalo entre
  // dentes fica enorme). A divisao original saturava sozinha; aqui, sem esta
  // guarda, o estouro daria um valor pequeno e ERRADO que passaria batido.
  if (intervalo_us >= limite_entrada_us) {
    return TEMPO_CADA_GRAU_MAX_US;
  }

  return (intervalo_us * reciproco) >> GRAU_RECIPROCO_SHIFT;
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

// ---- DEBUG TEMPORARIO: pulso de medicao da duracao da ISR do dente ----
// O pino 7 (ign4) fica ALTO durante a execucao da interrupcao e baixo fora
// dela. No analisador logico, a LARGURA de cada pulso e a duracao exata
// daquela interrupcao, e o espaco entre pulsos mostra o que sobra pro resto
// do sistema (ISRs do Timer1 + loop).
//
// O pino 7 esta livre na configuracao de 6 cilindros em centelha perdida, que
// usa so 3 canais fisicos (pinos 4, 5 e 6) - ver quantidade_canais_ignicao_fisicos.
// Ja e configurado como OUTPUT no setup.
//
// Por que nao usar micros() como antes: ele tem resolucao de 4us e custa ~4us
// por chamada, ou seja, o instrumento distorcia o que media. Aqui o pino e
// constante em tempo de compilacao, entao o compilador emite SBI/CBI - 2
// ciclos, atomico, sem cli(). Custo total ~0,25us.
//
// Medido com alvo 1: a ISR do dente e 53us tipicos, 64,3us de maximo absoluto
// em 204 mil pulsos, sem nenhum pico - 37% da CPU a 7000rpm. Nao e o gargalo.
// Alvo agora e 2, as ISRs de comparacao do Timer1, onde deve estar o resto.
//
// Alvo 2 tambem mediu: as ISRs do Timer1 custam so 1-6% da CPU. Somadas ao
// dente dao 40% a 7000rpm - sobra 60% livre, nao ha saturacao. O que os dados
// mostraram foi outra coisa: os pulsos do Timer1 caem de 8,0 por volta (o
// esperado) para 2,1 a 7000rpm, ou seja os eventos param de ser AGENDADOS.
// Alvo 3 mede exatamente isso: o tempo entre o gap e o instante em que o
// agendamento fica armado. Se esse atraso passar de (angulo da centelha menos
// o dwell), o evento nasce vencido e e cancelado. A 7000rpm essa folga e de
// apenas ~2,07ms.
//
// Alvo 3 tambem ja mediu: a latencia entre o gap e o agendamento armado e de
// ~1,1ms na mediana e ate ~2,9ms no p99. Nao era a causa das falhas.
//
// DESLIGADO por padrao. O pino 7 e o ign4, uma saida de ignicao real - ele so
// esta livre porque a configuracao de 6 cilindros em centelha perdida usa 3
// canais. Codigo de depuracao nao deve ficar chaveando pino de bobina: com uma
// quarta bobina ligada, ela receberia pulso a cada gap.
//
//   0 = desligado   1 = ISR do dente   2 = ISRs do Timer1   3 = atraso do agendamento
#define DEBUG_PULSO_ISR_ALVO 0

#if DEBUG_PULSO_ISR_ALVO && defined(__AVR_ATmega328P__)
  #define PULSO_ALTO()  (PORTD |= _BV(PD7))
  #define PULSO_BAIXO() (PORTD &= ~_BV(PD7))
#else
  #define PULSO_ALTO()
  #define PULSO_BAIXO()
#endif

#if DEBUG_PULSO_ISR_ALVO == 1
  #define PULSO_DENTE_ALTO()  PULSO_ALTO()
  #define PULSO_DENTE_BAIXO() PULSO_BAIXO()
#else
  #define PULSO_DENTE_ALTO()
  #define PULSO_DENTE_BAIXO()
#endif

#if DEBUG_PULSO_ISR_ALVO == 2
  #define PULSO_TIMER1_ALTO()  PULSO_ALTO()
  #define PULSO_TIMER1_BAIXO() PULSO_BAIXO()
#else
  #define PULSO_TIMER1_ALTO()
  #define PULSO_TIMER1_BAIXO()
#endif

// Alvo 3: sobe no gap (dentro da interrupcao, no instante exato em que o
// tick de referencia e capturado) e desce quando agendar_eventos_motor_timer1
// termina, no loop(). A largura do pulso e o atraso total ate os eventos
// ficarem armados - latencia do loop mais a duracao do proprio calculo.
#if DEBUG_PULSO_ISR_ALVO == 3
  #define PULSO_AGENDA_ALTO()  PULSO_ALTO()
  #define PULSO_AGENDA_BAIXO() PULSO_BAIXO()
#else
  #define PULSO_AGENDA_ALTO()
  #define PULSO_AGENDA_BAIXO()
#endif

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
    }
    return;
  }
  rejeicoes_dente_consecutivas = 0;

  ultimo_tempo_interrupcao = tempo_agora;
  ultimo_pulso_rpm_us = tempo_agora;

  qtd_leitura++;
  tempo_atual = tempo_agora;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);

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

  // Os limiares de gap so sao calculados quando ha chance real de ser gap.
  // Antes as duas multiplicacoes de 32 bits rodavam em TODO dente e eram
  // descartadas em ~74% deles, ja que a validacao de posicao acima rejeita
  // qualquer gap antes de 75% da volta. Como a condicao de posicao entrava no
  // mesmo E logico, adiantar o teste nao muda o resultado - so evita o
  // trabalho. Isso vale muito dentro da interrupcao do dente, que e o
  // gargalo do RPM maximo.
  bool gap_detectado = false;
  if (posicao_gap_plausivel &&
      amostras_intervalo_validas >= 3 &&
      intervalo_dente_referencia_us > 0) {
    unsigned long fator_gap_min_num = (rpm < rpm_partida)
                                          ? GAP_FATOR_MIN_PARTIDA_NUM(qtd_dente_faltante)
                                          : GAP_FATOR_MIN_FUNC_NUM(qtd_dente_faltante);
    unsigned long gap_minimo_us =
        (intervalo_dente_referencia_us * fator_gap_min_num) >> GAP_FATOR_MIN_SHIFT;

    if (intervalo_tempo_entre_dente > gap_minimo_us) {
      // gap_maximo so importa se o limiar minimo ja passou - segundo produto
      // de 32 bits evitado no caso comum.
      unsigned long gap_maximo_us =
          intervalo_dente_referencia_us * ((unsigned long)(qtd_dente_faltante + GAP_FATOR_MAX_MARGEM_X));
      gap_detectado = (intervalo_tempo_entre_dente < gap_maximo_us);
    }
  }

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
      PULSO_AGENDA_ALTO();
    }

    posicao_atual_sensor = 0;

  } else {
    posicao_atual_sensor++;
    if (grau_cada_dente > 0) {
      unsigned long tempo_instante_grau = dividir_por_grau_cada_dente(intervalo_tempo_entre_dente);
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
void leitor_sensor_roda_fonica() {
  PULSO_DENTE_ALTO();
  decoder_roda_fonica_padrao();
  PULSO_DENTE_BAIXO();
}
