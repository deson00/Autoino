static const uint16_t TIMER1_TICK_US = 4;
static const uint16_t TIMER1_MIN_DELTA_TICKS = 2;
static const uint8_t TIMER1_MAX_REPLAN_LOOPS = 32;
// Margem de seguranca entre ler o tick atual e escrever OCR1A/OCR1B: cobre so
// o tempo real de varrer os poucos canais + escrever o registrador (bem menos
// de 1 tick na pratica). O valor antigo (20 ticks = 80us) era bem mais folgado
// que o necessario e fazia o caminho caro (reprocessar na hora, cascata)
// disparar com mais frequencia do que precisava, principalmente em RPM alto.
static const uint16_t TIMER1_MARGEM_DEADLOCK_TICKS = 6;

static inline uint32_t us_para_ticks_timer1(unsigned long tempo_us) {
	if (tempo_us == 0) {
		return TIMER1_MIN_DELTA_TICKS;
	}

	uint32_t ticks = (uint32_t)((tempo_us + (TIMER1_TICK_US >> 1)) / TIMER1_TICK_US);
	if (ticks < TIMER1_MIN_DELTA_TICKS) {
		ticks = TIMER1_MIN_DELTA_TICKS;
	}
	return ticks;
}

static inline bool tick_ja_passou(uint32_t tick_atual, uint32_t tick_evento) {
	return (int32_t)(tick_atual - tick_evento) >= 0;
}

static inline uint32_t delta_tick_evento(uint32_t tick_atual, uint32_t tick_evento) {
	if (tick_ja_passou(tick_atual, tick_evento)) {
		return 0;
	}
	return tick_evento - tick_atual;
}

static inline uint32_t alinhar_tick_para_futuro(uint32_t tick_evento, uint32_t tick_atual, uint32_t periodo_ticks) {
	if (periodo_ticks == 0) {
		return tick_atual + TIMER1_MIN_DELTA_TICKS;
	}

	if (!tick_ja_passou(tick_atual, tick_evento)) {
		return tick_evento;
	}

	uint32_t atraso = tick_atual - tick_evento;
	uint32_t saltos = (atraso / periodo_ticks) + 1;
	return tick_evento + (saltos * periodo_ticks);
}

static inline uint32_t alinhar_tick_para_futuro_com_margem(uint32_t tick_evento, uint32_t tick_atual, uint32_t periodo_ticks, uint32_t margem_ticks) {
	uint32_t limite = tick_atual + margem_ticks;

	if (periodo_ticks == 0) {
		return limite + TIMER1_MIN_DELTA_TICKS;
	}

	if (!tick_ja_passou(limite, tick_evento)) {
		return tick_evento;
	}

	uint32_t atraso = limite - tick_evento;
	uint32_t saltos = (atraso / periodo_ticks) + 1;
	return tick_evento + (saltos * periodo_ticks);
}

// (contadores de estouro de replan removidos: mediram 0 em todos os testes,
// descartando a hipotese de canal orfao por estouro de tentativas)

static inline uint32_t ler_tick32_timer1() {
	uint32_t overflow_snapshot;
	uint16_t contador_snapshot;
	uint8_t houve_overflow_pendente;

	uint8_t sreg = SREG;
	cli();
	overflow_snapshot = timer1_overflow_count;
	contador_snapshot = TCNT1;
	houve_overflow_pendente = (TIFR1 & (1 << TOV1));
	if (houve_overflow_pendente && contador_snapshot < 65535) {
		overflow_snapshot++;
	}
	SREG = sreg;

	return (overflow_snapshot << 16) | contador_snapshot;
}

// So desarma. Uma tentativa anterior de "desligar tudo que estiver ligado antes
// de desarmar" foi medida e reprovada: matava a centelha. Nos 60 casos do log
// a bobina era desligada 0,18ms depois de ligar - dwell nenhum, centelha
// nenhuma. Quem chega aqui com saida ligada tem outro problema, e a solucao e
// nao chegar aqui (ver o rearme no estouro de tentativas).
// Verdadeiro enquanto agendar_eventos_motor_timer1 esta rodando. A ISR do
// compare A pode disparar nesse meio (ela NAO e mascarada - ver abaixo), e
// nesse caso ela faz so o trabalho de DESLIGAR: chamar atualizar_compare_b_ligar
// ali seria reentrar no mesmo planejamento que o agendador esta fazendo. O
// proprio agendador chama essa funcao no fim, entao nada se perde.
volatile bool agendador_em_execucao = false;

// Canais em REGIME ADIANTADO: a centelha deles esta agendada uma referencia a
// frente, porque o angulo alvo fica perto demais da referencia para o dwell
// caber a partir dela.
//
// Um canal em regime adiantado NAO e agendado pelo gap - ele e rearmado pelo
// loop logo depois de disparar, e por isso precisa sobreviver a limpeza do
// gap. Um bit por canal; ha no maximo 8.
volatile uint8_t ignicao_regime_adiantado = 0;

// Enrolar uma referencia so faz sentido se a referencia inteira couber na
// janela do comparador de 16 bits (65536 ticks = 262ms). Abaixo disso o
// comparador casa cedo e a ISR precisa rearmar varias vezes ate o evento -
// funciona, mas nao ha por que pagar isso em partida. 60000 ticks = 240ms,
// que no comando (uma volta de sensor = 720 graus) e ~500rpm de virabrequim.
// A Speeduino tem a guarda equivalente em setIgnitionScheduleDuration:
// angleToTime(CRANK_ANGLE_MAX_IGN) < MAX_TIMER_PERIOD.
static const uint32_t TIMER1_PERIODO_ADIANTA_MAX_TICKS = 60000UL;

#if DEBUG_PULSO_ISR_ALVO == 6
// Pulso so quando ha bobina ligada: desarmar sem bobina ligada e legitimo.
//   15us -> desabilitar_timer1_compare_a()  (varredura nao achou / estouro)
//   40us -> mascara do agendador            (a que eu introduzi)
static inline void pulso_quem_desarmou(uint8_t local) {
	bool ligada = false;
	for (byte i = 0; i < 8; i++) {
		if (ign_acionado[i]) { ligada = true; break; }
	}
	if (!ligada) return;
	PULSO_ALTO();
	if (local == 1) _delay_us(15); else _delay_us(40);
	PULSO_BAIXO();
}
#else
#define pulso_quem_desarmou(x)
#endif

static inline void desabilitar_timer1_compare_a() {
#if DEBUG_PULSO_ISR_ALVO == 4
	// Desarmar o compare A com bobina ligada = ninguem mais vai desliga-la.
	for (byte i = 0; i < 8; i++) {
		if (ign_acionado[i]) {
			PULSO_ORFAO_ALTO();
			break;
		}
	}
#endif
	TIMSK1 &= ~(1 << OCIE1A);
	pulso_quem_desarmou(1);
}

static inline void desabilitar_timer1_compare_b() {
	TIMSK1 &= ~(1 << OCIE1B);
}

static void atualizar_compare_b_ligar();
static void atualizar_compare_a_desligar();
static inline void agendar_injecao_canal(int i, uint32_t tick_atual);
static inline bool processar_cortes_vencidos(uint32_t tick_atual);

// Tempo entre REFERENCIAS angulares consecutivas, em ticks.
//
// Na roda fonica com dente de falha a referencia e o gap, que aparece uma vez
// por volta do sensor - dai 360 graus. No modo sem dente de falha cada pulso e
// uma referencia, e elas se repetem a cada grau_cada_dente graus (60 num
// distribuidor de 6 cilindros, 360 num volante de um dente). Usar 360 fixo la
// empurrava o evento uma volta inteira do sensor em vez de um pulso.
static inline uint32_t ticks_entre_referencias() {
	unsigned long graus = sensor_sem_falha() ? (unsigned long)grau_cada_dente : 360UL;
	return us_para_ticks_timer1(graus * tempo_cada_grau);
}

static inline void limpar_ignicoes_pendentes_nao_acionadas() {
	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		// Canal em regime adiantado nao e limpo: o agendamento dele aponta para
		// a referencia SEGUINTE, e apaga-lo aqui obrigaria a replaneja-lo a
		// partir desta referencia - de onde, por definicao, ele nao cabe.
		if (ignicao_regime_adiantado & (uint8_t)(1U << i)) {
			continue;
		}
		if (ignicao_agendada[i] && !ign_acionado[i] && !captura_dwell[i]) {
			ignicao_agendada[i] = false;
		}
	}
}

static inline uint32_t calcular_tick_fim_dwell_futuro(unsigned long tempo_ignicao_us, uint32_t tick_atual, uint32_t dwell_ticks, bool *enrolou) {
	// tick_base_sincronismo e a ORIGEM ANGULAR da volta (instante do gap).
	// Uma tentativa anterior de trocar essa ancora pelo tempo real na funcao
	// INTEIRA foi desastrosa (cobertura 100%->50% e morte em 4600rpm) - e
	// tambem carregava o bug de cancelamento cruzado, ja corrigido. Aqui o
	// tempo real entra em um unico ponto, apenas na decisao de "cabe o dwell?".
	uint32_t tick_fim_dwell = tick_base_sincronismo + us_para_ticks_timer1(tempo_ignicao_us);

	if (tempo_cada_grau > 0) {
		uint32_t periodo_ticks_360 = ticks_entre_referencias();

		if (tick_fim_dwell == tick_atual) {
			tick_fim_dwell += TIMER1_MIN_DELTA_TICKS;
		}

		if (tick_atual == tick_base_sincronismo) {
			// Primeiro agendamento da volta. Se o angulo alvo estiver perto
			// demais da referencia para caber o dwell inteiro, comeca a
			// carregar AGORA - centelha retardada e melhor que centelha
			// nenhuma, e empurrar pra volta seguinte derrubaria a desta volta.
			//
			// O "agora" e lido do timer, nao herdado de tick_atual: aqui
			// tick_atual e o instante do GAP, e esta funcao roda no loop() com
			// atraso variavel (medido em ~1,1ms de mediana). Usar o instante do
			// gap como se fosse o presente fazia o evento nascer VENCIDO, e
			// entao reagendar_ignicao_se_dwell_ficou_curto o cancelava - por
			// isso o canal mais proximo da referencia (ign1 na 60-2, que fica a
			// poucos graus do gap) perdia centelhas ao subir de rotacao.
			//
			// So esta comparacao usa o tempo real. O restante da funcao continua
			// ancorado em tick_base_sincronismo, que e a origem ANGULAR - e o
			// agendamento segue sendo limpo e recalculado a cada gap, sem
			// agendamento nenhum rodando livre entre voltas.
			// Quando o dwell inteiro nao cabe: MANTEM O ANGULO e deixa o dwell
			// encurtar. Antes preservava o dwell e empurrava a centelha, e isso
			// custava caro no primeiro canal depois do gap.
			//
			// Medido em motor real (60-2 no virabrequim, 6 cilindros, so
			// ignicao), centelha da ign1 a 56 graus do gap e agendador rodando
			// ~2,0ms depois dele:
			//
			//   rpm      ign1        ign2 e ign3
			//   1700     56,6 graus   176 / 297  (planos)
			//   2100     63,6         176 / 297
			//   2500     71,6         176 / 297
			//   2800     80,3 graus   176 / 297
			//
			// Ate 1900rpm o dwell comeca depois de 2,3ms e cabe; acima, comeca
			// antes de o agendador rodar e o empurrao entra - dai o joelho em
			// 1900 e os 24 graus de atraso em 2800. ign2 e ign3, longe do gap,
			// ficam planos: a prova de que nao era o avanco, que moveria os
			// tres juntos.
			//
			// Vinte e quatro graus de atraso e pior que meio dwell. Abaixo do
			// piso, porem, a centelha nao sai, e ai atrasada volta a ser melhor
			// que nenhuma - por isso o comportamento antigo continua como
			// ultimo recurso.
			//
			// TERCEIRA opcao, que so existe desde que o rearme saiu do gap:
			// ENROLAR uma referencia. Em vez de empurrar a centelha ou cortar o
			// dwell, agenda o MESMO angulo uma volta a frente, onde ha a volta
			// inteira de pista para o dwell. E o que a Speeduino faz em
			// _calculateCoilChargeAngle (o angulo de carga nasce somando
			// CRANK_ANGLE_MAX_IGN quando fica negativo) e em
			// _calculateAngularTime (while(delta < 0) delta += maxAngle).
			//
			// O comentario que estava aqui dizia que empurrar para a volta
			// seguinte "derrubaria a desta volta". Isso so vale se o canal for
			// agendado uma unica vez por volta, no gap: dai ele perderia uma
			// volta sim, outra nao. Com o rearme no loop
			// (rearmar_ignicoes_adiante) o canal e reagendado assim que
			// dispara, e o regime permanente e uma centelha por volta. O custo
			// e uma unica centelha, na primeira volta apos o sincronismo.
			//
			// O criterio de ENTRADA no regime adiantado nao e "nao coube
			// agora" - e "nao cabe nunca": angulo menor que o proprio dwell.
			//
			// A primeira versao usava o disponivel medido, que carrega a
			// latencia do loop. Medido em bancada numa varredura de 1015 a
			// 5989 rpm: um pico de latencia de ~1,8ms fazia o canal entrar no
			// regime a 3000rpm, onde o angulo de 31 graus da 3,2ms e o dwell
			// de 2,9ms cabe folgado. Resultado, 30 trocas de regime entre 3200
			// e 2700 rpm na desaceleracao, alternando quase volta a volta, com
			// a centelha saindo 5 a 8 graus adiantada nas voltas enroladas.
			//
			// Com o criterio no angulo ha uma consequencia estrutural que vale
			// mais que o criterio em si: o inicio do dwell enrolado cai em
			// (angulo - dwell) relativo ao gap seguinte, que e NEGATIVO por
			// definicao da propria condicao. Ou seja, a bobina sempre ja esta
			// carregando quando o gap chega, e o agendador do gap encontra o
			// canal ocupado. A corrida entre o rearme do loop e o gap deixa de
			// existir em vez de ser arbitrada.
			uint32_t angulo_ticks = us_para_ticks_timer1(tempo_ignicao_us);

			if (angulo_ticks < dwell_ticks &&
			    periodo_ticks_360 > 0 &&
			    periodo_ticks_360 <= TIMER1_PERIODO_ADIANTA_MAX_TICKS) {
				// O dwell nao cabe entre a referencia e o alvo em NENHUMA
				// circunstancia - nem com latencia zero. Enrola.
				//
				// Esta decisao nao consulta o tempo real de proposito. A
				// versao anterior exigia tambem "disponivel < piso", que
				// embute a latencia do loop, e o efeito medido em bancada
				// (3094 voltas, 1013 a 5989 rpm) foi uma populacao bimodal:
				// acima de 3400rpm, onde o angulo ja e menor que o dwell, o
				// canal entrava no regime so quando a latencia dava um pico -
				// 693 das 3094 centelhas continuavam saindo com menos de
				// 2,5ms de dwell, na mesma rotacao em que outras saiam com
				// 2,95ms.
				tick_fim_dwell += periodo_ticks_360;
				if (enrolou != NULL) {
					*enrolou = true;
				}
			} else {
				uint32_t agora = ler_tick32_timer1();
				int32_t disponivel = (int32_t)(tick_fim_dwell - agora);
				if (disponivel < (int32_t)dwell_ticks) {
					uint32_t piso = (dwell_ticks * DWELL_MINIMO_UTIL_PCT) / 100UL;
					if (disponivel < (int32_t)piso) {
						// Cabe pela geometria, so nao coube por latencia (ou a
						// referencia e longa demais para o comparador, na
						// partida): comportamento antigo, centelha atrasada.
						tick_fim_dwell = agora + dwell_ticks;
					}
					// senao: nao mexe. tick_inicio_dwell nasce no passado e
					// processar_ligamentos_vencidos liga a bobina na hora, com
					// o dwell reduzido ao que couber.
				}
			}
		} else {
			// Caminho do REARME (chamado apos o evento disparar, com o tempo
			// real). Aqui o proximo evento deste canal esta, por definicao,
			// exatamente um periodo a frente - qualquer alvo a menos disso e
			// duplicata na mesma volta.
			//
			// A margem antiga era dwell + delta, uns poucos graus. Insuficiente:
			// o rearme recalcula o angulo com o tempo_cada_grau ATUAL, e em
			// DESACELERACAO esse valor cresce, jogando o alvo recalculado mais
			// adiante na mesma volta - ainda no futuro, logo aceito, e o canal
			// dispara de novo.
			//
			// Medido em bancada com roda 12-1 no comando, rotacao variando:
			// duplicatas em 16-43% das voltas em desaceleracao contra 0-2% em
			// aceleracao. Meio periodo e folgado o bastante para rejeitar
			// qualquer alvo da volta corrente e permissivo o bastante para
			// aceitar o legitimo, que fica a um periodo inteiro.
			tick_fim_dwell = alinhar_tick_para_futuro_com_margem(tick_fim_dwell, tick_atual, periodo_ticks_360, periodo_ticks_360 >> 1);
		}
	} else if (tick_ja_passou(tick_atual + dwell_ticks + TIMER1_MIN_DELTA_TICKS, tick_fim_dwell)) {
		tick_fim_dwell = tick_atual + dwell_ticks + TIMER1_MIN_DELTA_TICKS;
	}

	return tick_fim_dwell;
}

static inline void agendar_ignicao_canal(int i, uint32_t tick_atual) {
	if (status_corte != 0) {
		ignicao_agendada[i] = false;
		return;
	}

	if (ign_acionado[i] || captura_dwell[i] || ignicao_agendada[i]) {
		return;
	}

	// Canal em regime adiantado pertence ao rearme do loop, nao ao gap. Sem
	// esta porta, um canal que estivesse livre no instante do gap era
	// replanejado por ele a partir desta referencia - de onde nao cabe - e o
	// regime alternava volta a volta. O rearme sempre devolve o bit quando o
	// canal deixa de precisar (ver rearmar_ignicoes_adiante), entao nao ha
	// como um canal ficar preso aqui.
	if (ignicao_regime_adiantado & (uint8_t)(1U << i)) {
		return;
	}

	uint32_t dwell_ticks = us_para_ticks_timer1(dwell_bobina);
	calcula_grau_ignicao(i);
	bool enrolou = false;
	uint32_t tick_fim_dwell = calcular_tick_fim_dwell_futuro(tempo_proxima_ignicao[i], tick_atual, dwell_ticks, &enrolou);

	uint32_t tick_inicio_dwell = tick_fim_dwell - dwell_ticks;

	ignicao_tick_ligar[i] = tick_inicio_dwell;
	ignicao_tick_desligar[i] = tick_fim_dwell;
	ignicao_agendada[i] = true;
	if (enrolou) {
		ignicao_regime_adiantado |= (uint8_t)(1U << i);
	}
}

// Rearme dos canais em regime adiantado, fora do gap.
//
// Um canal so e agendado no gap. Quando ele acaba de disparar, o proximo alvo
// alcancavel a partir do gap SEGUINTE ja e a volta depois dele - entao esperar
// pelo gap custaria uma volta inteira, e o canal dispararia uma volta sim,
// outra nao. E o mesmo motivo pelo qual a Speeduino chama setIgnitionChannels
// a cada passada do loop e nao uma vez por referencia.
//
// Roda so para os canais marcados, e so quando eles estao livres - ou seja,
// alguns bytes de teste nas passadas em que nao ha nada a fazer.
//
// A ancora continua sendo o ultimo gap REAL (tick_base_sincronismo) mais um
// periodo estimado, nao uma cadeia de estimativas: o erro nao acumula, e o
// proprio alinhamento com margem de meio periodo e o mesmo ja usado (e ja
// testado contra duplicata em desaceleracao) no caminho de rearme antigo.
static void rearmar_ignicoes_adiante() {
	if (ignicao_regime_adiantado == 0) {
		return;
	}

	if (tipo_ignicao_sequencial != 0 || revolucoes_sincronizada < 1 || status_corte != 0 || tempo_cada_grau == 0) {
		ignicao_regime_adiantado = 0;
		return;
	}

	uint32_t periodo_ticks = ticks_entre_referencias();
	if (periodo_ticks == 0 || periodo_ticks > TIMER1_PERIODO_ADIANTA_MAX_TICKS) {
		ignicao_regime_adiantado = 0;
		return;
	}

	uint32_t dwell_ticks = us_para_ticks_timer1(dwell_bobina);
	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (!(ignicao_regime_adiantado & (uint8_t)(1U << i))) {
			continue;
		}
		if (ignicao_agendada[i] || ign_acionado[i] || captura_dwell[i]) {
			continue; // ainda ocupado com o ciclo corrente
		}

		calcula_grau_ignicao(i);
		uint32_t angulo_ticks = us_para_ticks_timer1(tempo_proxima_ignicao[i]);
		uint32_t alvo = tick_base_sincronismo + angulo_ticks;

		// Saida do regime: se o angulo ja folga o dwell com sobra, o gap volta
		// a dar conta com ancora exata, que e sempre melhor que a estimada.
		// A folga de 25% e histerese - sem ela o canal entraria e sairia do
		// regime a cada volta na fronteira.
		if (angulo_ticks > (dwell_ticks + (dwell_ticks >> 2))) {
			ignicao_regime_adiantado &= (uint8_t)~(1U << i);
			continue;
		}

		uint32_t agora = ler_tick32_timer1();
		alvo = alinhar_tick_para_futuro_com_margem(alvo, agora, periodo_ticks, periodo_ticks >> 1);

		uint8_t sreg = SREG;
		cli();
		ignicao_tick_ligar[i] = alvo - dwell_ticks;
		ignicao_tick_desligar[i] = alvo;
		ignicao_agendada[i] = true;
		atualizar_compare_b_ligar();
		atualizar_compare_a_desligar();
		SREG = sreg;
	}
}

static inline void recalcular_ignicao_canal_por_dente(int i, uint32_t tick_atual) {
	if (status_corte != 0 || tempo_cada_grau == 0 || grau_cada_dente == 0) {
		return;
	}

	// Canal em regime adiantado tem alvo uma referencia a frente; o refino por
	// dente raciocina dentro da volta corrente e o traria de volta para ela.
	if (ignicao_regime_adiantado & (uint8_t)(1U << i)) {
		return;
	}

	if (!ignicao_agendada[i] || ign_acionado[i] || captura_dwell[i]) {
		return;
	}

	uint32_t dwell_ticks = us_para_ticks_timer1(dwell_bobina);
	int angulo_alvo = normalizar_angulo_minimo_zero(calcular_angulo_ignicao_indice(i));
	int angulo_sensor_atual = normalizar_angulo_minimo_zero(posicao_atual_sensor * (int)grau_cada_dente);
	int graus_ate_evento = angulo_alvo - angulo_sensor_atual;
	if (graus_ate_evento <= 0) {
		graus_ate_evento += 360;
	}

	uint32_t ticks_ate_fim = us_para_ticks_timer1((unsigned long)graus_ate_evento * tempo_cada_grau);
	uint32_t tick_fim_dwell = tick_atual + ticks_ate_fim;
	if (tick_ja_passou(tick_atual + dwell_ticks + TIMER1_MIN_DELTA_TICKS, tick_fim_dwell)) {
		return;
	}

	ignicao_tick_ligar[i] = tick_fim_dwell - dwell_ticks;
	ignicao_tick_desligar[i] = tick_fim_dwell;
}

static inline void recalcular_injecao_canal_por_dente(int i, uint32_t tick_atual) {
	if (tempo_cada_grau == 0 || grau_cada_dente == 0) {
		return;
	}

	if (!injecao_agendada[i] || inj_acionado[i] || captura_req_fuel[i]) {
		return;
	}

	int angulo_alvo = normalizar_angulo_minimo_zero(calcular_angulo_injecao_indice(i));
	int angulo_sensor_atual = normalizar_angulo_minimo_zero(posicao_atual_sensor * (int)grau_cada_dente);
	int graus_ate_evento = angulo_alvo - angulo_sensor_atual;
	if (graus_ate_evento <= 0) {
		graus_ate_evento += 360;
	}

	uint32_t ticks_ate_inicio = us_para_ticks_timer1((unsigned long)graus_ate_evento * tempo_cada_grau);
	uint32_t tick_inicio_injecao = tick_atual + ticks_ate_inicio;
	uint32_t tempo_injecao_ticks = us_para_ticks_timer1(tempo_injecao);

	injecao_tick_ligar[i] = tick_inicio_injecao;
	injecao_tick_desligar[i] = tick_inicio_injecao + tempo_injecao_ticks;
}

// Refinamento por dente processa 1 canal de ignicao por vez, alternando em
// sequencia, em vez de recalcular todos a cada chamada. O calculo cheio de
// todos os canais continua acontecendo 1x por volta no evento de gap
// (agendar_eventos_motor_timer1) - isto aqui e so a correcao fina entre um
// gap e outro, entao nao ha problema em espacar um pouco mais por canal.
volatile byte proximo_canal_ignicao_recalculo = 0;

void atualizar_agendamentos_ignicao_por_dente() {
	if (tipo_ignicao_sequencial != 0 || revolucoes_sincronizada < 1 ||
	    (local_rodafonica != 1 && local_rodafonica != 2) || tempo_cada_grau == 0) {
		return;
	}

	uint8_t sreg = SREG;
	cli();
	uint32_t tick_atual = ler_tick32_timer1();
	bool algo_desligou = processar_cortes_vencidos(tick_atual);

	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	if (eventos_ignicao > 0) {
		if (proximo_canal_ignicao_recalculo >= eventos_ignicao) {
			proximo_canal_ignicao_recalculo = 0;
		}
		recalcular_ignicao_canal_por_dente(proximo_canal_ignicao_recalculo, tick_atual);
		proximo_canal_ignicao_recalculo++;
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		recalcular_injecao_canal_por_dente(i, tick_atual);
	}

	// A segunda chamada a processar_cortes_vencidos(tick_atual) foi removida:
	// era redundante, checava o mesmo tick_atual ja avaliado acima, nunca
	// encontrava nada novo.
	// As duas, sempre - mesmo motivo da correcao em agendar_eventos_motor_timer1.
	//
	// A condicao antiga raciocinava sobre recalcular_ignicao_canal_por_dente,
	// que de fato so mexe em canais ainda nao ligados. Mas quem liga bobina
	// aqui e atualizar_compare_b_ligar(), pelo caminho de prevencao de
	// deadlock, e isso acontece DEPOIS de algo_desligou ter sido calculado.
	// Bobina ligada sem o compare A armado fica presa ate a protecao de dwell.
	atualizar_compare_b_ligar();
	atualizar_compare_a_desligar();
	(void)algo_desligou;

	SREG = sreg;
}

static inline void agendar_injecao_canal(int i, uint32_t tick_atual) {
	if (inj_acionado[i] || captura_req_fuel[i] || injecao_agendada[i]) {
		return;
	}

	calcula_grau_injetor(i);

	unsigned long tempo_injecao_inicio_us = tempo_proxima_injecao[i];
	uint32_t tick_inicio_injecao = tick_base_sincronismo + us_para_ticks_timer1(tempo_injecao_inicio_us);

	if (tempo_cada_grau > 0) {
		uint32_t periodo_ticks_360 = ticks_entre_referencias();

		if (tick_inicio_injecao == tick_atual) {
			tick_inicio_injecao += TIMER1_MIN_DELTA_TICKS;
		}

		// Mesma protecao da ignicao contra duplicata em desaceleracao. Antes
		// usava alinhar_tick_para_futuro, SEM margem nenhuma: bastava o alvo
		// recalculado ficar um tick a frente para ser aceito na mesma volta.
		// Com 6 eventos por volta no modo comando, todos duplicavam juntos -
		// medimos 12 injecoes em voltas que deviam ter 6.
		if (tick_atual == tick_base_sincronismo) {
			tick_inicio_injecao = alinhar_tick_para_futuro(tick_inicio_injecao, tick_atual, periodo_ticks_360);
		} else {
			tick_inicio_injecao = alinhar_tick_para_futuro_com_margem(tick_inicio_injecao, tick_atual, periodo_ticks_360, periodo_ticks_360 >> 1);
		}
	} else if (tick_ja_passou(tick_atual, tick_inicio_injecao)) {
		tick_inicio_injecao = tick_atual + TIMER1_MIN_DELTA_TICKS;
	}

	uint32_t tick_fim_injecao = tick_inicio_injecao + us_para_ticks_timer1(tempo_injecao);
	injecao_tick_ligar[i] = tick_inicio_injecao;
	injecao_tick_desligar[i] = tick_fim_injecao;
	injecao_agendada[i] = true;
}

static inline bool reagendar_injecao_se_pulso_ficou_curto(int i, uint32_t tick_atual) {
	uint32_t tempo_injecao_ticks = us_para_ticks_timer1(tempo_injecao);
	uint32_t tempo_restante_ticks = delta_tick_evento(tick_atual, injecao_tick_desligar[i]);
	uint32_t pulso_minimo_util_ticks = (tempo_injecao_ticks * 80UL) / 100UL;
	if (pulso_minimo_util_ticks < TIMER1_MIN_DELTA_TICKS) {
		pulso_minimo_util_ticks = TIMER1_MIN_DELTA_TICKS;
	}

	if (tempo_restante_ticks >= pulso_minimo_util_ticks) {
		return false;
	}

	if (tempo_cada_grau == 0) {
		injecao_tick_ligar[i] = tick_atual + TIMER1_MIN_DELTA_TICKS;
		injecao_tick_desligar[i] = injecao_tick_ligar[i] + tempo_injecao_ticks;
		return true;
	}

	uint32_t periodo_ticks_360 = ticks_entre_referencias();
	injecao_tick_ligar[i] = alinhar_tick_para_futuro_com_margem(injecao_tick_ligar[i],
	                                                            tick_atual,
	                                                            periodo_ticks_360,
	                                                            tempo_injecao_ticks + TIMER1_MIN_DELTA_TICKS);
	injecao_tick_desligar[i] = injecao_tick_ligar[i] + tempo_injecao_ticks;
	return true;
}

static inline bool reagendar_ignicao_se_dwell_ficou_curto(int i, uint32_t tick_atual) {
	uint32_t dwell_ticks = us_para_ticks_timer1(dwell_bobina);
	uint32_t tempo_restante_ticks = delta_tick_evento(tick_atual, ignicao_tick_desligar[i]);
	uint32_t dwell_minimo_util_ticks = (dwell_ticks * DWELL_CANCELA_ABAIXO_PCT) / 100UL;
	if (dwell_minimo_util_ticks < TIMER1_MIN_DELTA_TICKS) {
		dwell_minimo_util_ticks = TIMER1_MIN_DELTA_TICKS;
	}

	if (tempo_restante_ticks >= dwell_minimo_util_ticks) {
		return false;
	}

	// Cancela APENAS este canal. Antes chamava limpar_ignicoes_pendentes_nao_acionadas(),
	// que apaga o agendamento de TODOS os canais - uma checagem por canal com
	// efeito global. Como os angulos de centelha ficam a 120 graus (213, 333 e
	// 93 apos o gap), o canal de 93 graus e sempre o apertado: o dwell de 3ms
	// ocupa 72 graus a 4000rpm e 126 graus a 7000rpm, entao o inicio do
	// carregamento dele cai antes do proprio gap. Ele falhava primeiro e
	// derrubava os outros dois junto, produzindo o padrao medido de tudo-ou-nada
	// (ou os 3 canais disparam na volta, ou nenhum) e cobertura identica nos
	// tres. A versao da injecao, reagendar_injecao_se_pulso_ficou_curto, sempre
	// mexeu so no canal recebido - era assimetria, nao intencao.
	ignicao_agendada[i] = false;
	return true;
}

static inline void processar_ligamentos_vencidos(uint32_t tick_atual) {
	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (ignicao_agendada[i] && !ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_ligar[i])) {
			if (reagendar_ignicao_se_dwell_ficou_curto(i, tick_atual)) {
				continue;
			}
			iniciar_dwell(i);
		}
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		if (injecao_agendada[i] && !inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_ligar[i])) {
			if (reagendar_injecao_se_pulso_ficou_curto(i, tick_atual)) {
				continue;
			}
			ligar_injetor(i);
		}
	}
}

static inline void limpar_ligamentos_vencidos_sem_acionar(uint32_t tick_atual) {
	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (ignicao_agendada[i] && !ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_ligar[i])) {
			ignicao_agendada[i] = false;
		}
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		if (injecao_agendada[i] && !inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_ligar[i])) {
			injecao_agendada[i] = false;
		}
	}
}

static inline bool processar_cortes_vencidos(uint32_t tick_atual) {
	bool algo_desligou = false;
	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_desligar[i])) {
			desligar_dwell(i);
			ignicao_agendada[i] = false;
			algo_desligou = true;
			// SEM REARME. O canal que acabou de disparar so volta a disparar uma
			// volta depois, e o gap sempre chega antes disso e reagenda tudo -
			// entao o rearme nao adianta nada e so cria risco.
			//
			// O risco medido: o rearme deslocava o alvo por um periodo estimado
			// com o tempo_cada_grau atual. Desacelerando esse valor esta
			// atrasado, o periodo sai CURTO, e o alvo deslocado dispara ANTES do
			// gap - escapando da limpeza que o cancelaria. So atinge canais de
			// angulo pequeno: no ign1 (centelha em 22 graus) a extra saia em 353
			// graus, colada no gap seguinte; o ign2 (207 graus) nunca duplicou.
			//
			// O modo virabrequim nunca teve rearme e nunca teve esse defeito.
			// Agora os dois modos agendam apenas na referencia.
		}
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		if (inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_desligar[i])) {
			desligar_injetor(i);
			injecao_agendada[i] = false;
			algo_desligou = true;
			// Sem rearme, mesmo motivo da ignicao (ver acima).
		}
	}
	return algo_desligou;
}

void setupTimer1() {
	noInterrupts();

	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	TCCR1B |= (1 << CS11) | (1 << CS10);

	TIFR1 = (1 << OCF1A) | (1 << OCF1B);
	TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1));
	timer1_overflow_count = 0;
	TIMSK1 |= (1 << TOIE1);

	interrupts();
}

void limpar_agendamentos_timer1() {
	uint8_t sreg = SREG;
	cli();
	ignicao_regime_adiantado = 0;
	for (int i = 0; i < 8; i++) {
		ignicao_agendada[i] = false;
		injecao_agendada[i] = false;
	}
	TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B));
	SREG = sreg;
}

void resetar_estado_agendamento_motor() {
	uint8_t sreg = SREG;
	cli();

	ignicao_regime_adiantado = 0;
	for (int i = 0; i < 8; i++) {
		ignicao_agendada[i] = false;
		injecao_agendada[i] = false;
		ign_acionado[i] = false;
		captura_dwell[i] = false;
		inj_acionado[i] = false;
		captura_req_fuel[i] = false;
	}

	TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B));
	TIFR1 = (1 << OCF1A) | (1 << OCF1B);

	SREG = sreg;

	if (modo_injecao != 0) {
		for (int i = 0; i < numero_injetor; i++) {
			digitalWrite(injecao_pins[i], LOW);
		}
	}
	byte canais_ignicao = quantidade_canais_ignicao_fisicos();
	for (int i = 0; i < canais_ignicao; i++) {
		digitalWrite(ignicao_pins[i], LOW);
	}
}

static void atualizar_compare_b_ligar() {
	uint8_t tentativas = 0;
	while (true) {
		if (tentativas++ >= TIMER1_MAX_REPLAN_LOOPS) {
			desabilitar_timer1_compare_b();
			break;
		}

		uint32_t agora = ler_tick32_timer1();
		bool encontrado = false;
		uint32_t menor_delta = 0xFFFFFFFFUL;
		uint32_t proximo_tick = 0;

		byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
		for (int i = 0; i < eventos_ignicao; i++) {
			if (ignicao_agendada[i] && !ign_acionado[i]) {
				uint32_t alvo = ignicao_tick_ligar[i];
				uint32_t delta = delta_tick_evento(agora, alvo);
				if (!encontrado || delta < menor_delta) {
					encontrado = true;
					menor_delta = delta;
					proximo_tick = alvo;
				}
			}
		}

		byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
		for (int i = 0; i < eventos_injecao; i++) {
			if (injecao_agendada[i] && !inj_acionado[i]) {
				uint32_t alvo = injecao_tick_ligar[i];
				uint32_t delta = delta_tick_evento(agora, alvo);
				if (!encontrado || delta < menor_delta) {
					encontrado = true;
					menor_delta = delta;
					proximo_tick = alvo;
				}
			}
		}

		if (!encontrado) {
			desabilitar_timer1_compare_b();
			break;
		}

		// Prevenção de deadlock do comparador de hardware (evento muito próximo ou já ficou pro passado):
		if (menor_delta < TIMER1_MARGEM_DEADLOCK_TICKS) {
			uint32_t tick_evento = agora + menor_delta;
			processar_ligamentos_vencidos(tick_evento);
			processar_cortes_vencidos(tick_evento);
			continue; // O evento acabou de ser consumido, recalcula o próximo!
		}

		OCR1B = (uint16_t)proximo_tick;
		TIFR1 = (1 << OCF1B);
		TIMSK1 |= (1 << OCIE1B);

		// Dupla checagem: Se o comparador girou e passou do valor DURANTE o bloco acima
		if (tick_ja_passou(ler_tick32_timer1(), proximo_tick)) {
			continue;
		}
		break; // Agendado com sucesso e margem de folga de hardware!
	}
}

static void atualizar_compare_a_desligar() {
	uint8_t tentativas = 0;
	while (true) {
		if (tentativas++ >= TIMER1_MAX_REPLAN_LOOPS) {
			// Estourar as tentativas NAO pode desarmar o compare A: se ha saida
			// ligada, desarmar significa que ninguem mais a desliga, e ela fica
			// carregando ate protege_dwell_maximo cortar.
			//
			// Em vez disso, arma para daqui a pouco e deixa a ISR reavaliar com
			// estado fresco. Custa uma interrupcao a mais e sai do laco.
			bool algo_ligado = false;
			byte n_ign = quantidade_eventos_ignicao_por_ciclo_sensor();
			for (byte i = 0; i < n_ign && !algo_ligado; i++) {
				if (ign_acionado[i]) algo_ligado = true;
			}
			byte n_inj = quantidade_eventos_injecao_por_ciclo_sensor();
			for (byte i = 0; i < n_inj && !algo_ligado; i++) {
				if (inj_acionado[i]) algo_ligado = true;
			}
			// 200us, nao TIMER1_MIN_DELTA_TICKS (2 ticks = 8us): rearmar a 8us
			// faz a ISR disparar antes de terminar o trabalho, e o resultado
			// medido foi tempestade de interrupcao - 429 bobinas presas a
			// 4000rpm contra 21 sem o rearme. Folga o bastante para a ISR rodar
			// inteira, curto o bastante para nao segurar bobina.
			if (algo_ligado) {
				OCR1A = (uint16_t)(ler_tick32_timer1() + us_para_ticks_timer1(200));
				TIFR1 = (1 << OCF1A);
				TIMSK1 |= (1 << OCIE1A);
			} else {
				desabilitar_timer1_compare_a();
			}
			break;
		}

		uint32_t agora = ler_tick32_timer1();
		bool encontrado = false;
		uint32_t menor_delta = 0xFFFFFFFFUL;
		uint32_t proximo_tick = 0;

		// Criterio: ign_acionado, e SO ele.
		//
		// Antes exigia tambem ignicao_agendada. Com isso, uma bobina ligada cujo
		// agendamento tivesse sido apagado ficava invisivel para a varredura,
		// que concluia "nao ha nada para desligar" e DESARMAVA o compare A. Como
		// e a propria ISR do compare A que se rearmaria, ela nunca mais
		// disparava: a bobina carregava ate protege_dwell_maximo cortar.
		//
		// Medido em bancada com pulso de depuracao na condicao exata (compare A
		// desarmado com bobina ligada): a 4000rpm, 302 dwells longos e 299 com o
		// pulso em cima. Em 3800 e 4200 rpm, 1 e 2 - faixa estreitissima, porque
		// so acontece quando o prazo do evento fica da mesma ordem da latencia
		// do loop e o codigo alterna entre dois caminhos.
		//
		// Quem manda em "precisa desligar" e o estado FISICO da bobina, nao o do
		// agendamento. O alvo continua valido: ignicao_tick_desligar[i] foi
		// escrito no mesmo agendamento que ligou o canal.
		byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
		for (int i = 0; i < eventos_ignicao; i++) {
			if (ign_acionado[i]) {
				uint32_t alvo = ignicao_tick_desligar[i];
				uint32_t delta = delta_tick_evento(agora, alvo);
				if (!encontrado || delta < menor_delta) {
					encontrado = true;
					menor_delta = delta;
					proximo_tick = alvo;
				}
			}
		}

		// Mesmo criterio da ignicao: bico aberto precisa fechar. Aqui o risco de
		// ficar preso e ainda pior - bico aberto e combustivel entrando.
		byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
		for (int i = 0; i < eventos_injecao; i++) {
			if (inj_acionado[i]) {
				uint32_t alvo = injecao_tick_desligar[i];
				uint32_t delta = delta_tick_evento(agora, alvo);
				if (!encontrado || delta < menor_delta) {
					encontrado = true;
					menor_delta = delta;
					proximo_tick = alvo;
				}
			}
		}

		if (!encontrado) {
			desabilitar_timer1_compare_a();
			break;
		}

		// Prevenção de deadlock pro desligamento de bobina/bico:
		if (menor_delta < TIMER1_MARGEM_DEADLOCK_TICKS) {
			processar_cortes_vencidos(agora + menor_delta);
			continue;
		}

		OCR1A = (uint16_t)proximo_tick;
		TIFR1 = (1 << OCF1A);
		TIMSK1 |= (1 << OCIE1A);

		if (tick_ja_passou(ler_tick32_timer1(), proximo_tick)) {
			continue;
		}
		break;
	}
}

void agendar_eventos_motor_timer1() {
	if (tipo_ignicao_sequencial != 0 || revolucoes_sincronizada < 1) {
		limpar_agendamentos_timer1();
		return;
	}

	atualizar_ajuste_pms_ignicao();

	// cli() GLOBAL, nao apenas mascarar OCIE1A/OCIE1B.
	//
	// Cheguei a trocar por TIMSK1 &= ~(OCIE1A|OCIE1B) achando que a interrupcao
	// do dente, sendo externa (EIMSK), nao disputaria estes vetores. Estava
	// ERRADO: abaixo de RECALCULO_AGENDAMENTO_RPM_MAXIMO a ISR do dente chama
	// atualizar_agendamentos_ignicao_por_dente(), que mexe nos MESMOS arrays -
	// processar_cortes_vencidos, os recalcular_* e atualizar_compare_b_ligar.
	// Sem o cli() ela entra no meio deste calculo e o agendamento sai duplicado.
	//
	// Medido em motor real (12-1 no comando) e reproduzido em bancada variando
	// a rotacao: abaixo de ~1000rpm quase metade das voltas saia com centelha
	// ou injecao a mais - ate 12 injecoes numa volta que deveria ter 6. Acima
	// disso, onde o reagendamento por dente nao roda, a injecao acertava 100%.
	// Em rotacao FIXA o problema nao aparece, e foi por isso que passou pelos
	// testes anteriores.
	// Acima de RECALCULO_AGENDAMENTO_RPM_MAXIMO a ISR do dente NAO chama
	// atualizar_agendamentos_ignicao_por_dente(), entao o conflito descrito
	// acima nao pode acontecer - e ali o cli() global sai caro demais.
	//
	// Medido em bancada com pulso de depuracao na propria ISR do dente
	// (DEBUG_PULSO_ISR_ALVO 1, 60-2 simulada ate 7000rpm): 96,5% dos dentes
	// com latencia alta caiam a menos de 100us de uma borda de ignicao, e os
	// dentes PERDIDOS se concentravam nas posicoes 2 a 6 da volta - que e
	// exatamente quando esta funcao roda, logo depois do gap. Resultado: a
	// contagem chegava ao gap com 55-57 dentes em vez de 58 (96,5% das voltas),
	// contagem_valida so aceita 57..59, e 3 invalidas seguidas zeravam
	// revolucoes_sincronizada - uma volta INTEIRA sem ignicao.
	//
	// A assinatura no log era inconfundivel e so faz sentido com esta causa:
	// a volta sem centelha era a unica com os 58 dentes contados certos
	// (99,9% delas), porque sem ignicao nao havia ISR roubando dente. O
	// firmware se auto-sabotava em ciclo - a ignicao estragava a contagem, a
	// contagem derrubava o sincronismo, e a perda de sincronismo desligava a
	// ignicao, que era o que consertava a contagem.
	//
	// Mascarar so OCIE1A/OCIE1B protege os mesmos arrays contra as ISRs do
	// Timer1 (que sao quem mais os toca) e deixa a interrupcao do dente entrar
	// na hora. Evento que vencer durante a mascara nao se perde: o flag fica em
	// TIFR1 e a ISR dispara assim que o bit volta.
	//
	// A margem de histerese evita alternar de regime na fronteira. rpm so e
	// escrito no loop (calcularRPM), nunca na interrupcao, entao esta decisao
	// nao muda no meio da secao critica.
	const bool proteger_contra_isr_do_dente =
		(rpm < (RECALCULO_AGENDAMENTO_RPM_MAXIMO + 200U));

	uint8_t sreg = SREG;
	uint8_t timsk_salvo;
	if (proteger_contra_isr_do_dente) {
		cli();
		timsk_salvo = TIMSK1;
	} else {
		cli();
		timsk_salvo = TIMSK1;
		// Mascara SO o OCIE1B.
		//
		// O compare A e quem DESLIGA a bobina, e silencia-lo com bobina ligada
		// e o pior estado possivel: ninguem mais a desliga, e ela carrega ate
		// protege_dwell_maximo cortar, a 1,5x o dwell.
		//
		// Medido em bancada a 4000rpm, com pulso identificando quem desarmava:
		// 86% dos desarmes com bobina ligada vinham DESTA mascara, e 74 dos 86
		// dwells presos tinham o pulso dela em cima - nenhum ficou sem pulso.
		// A mascara foi introduzida junto com o estreitamento da secao critica
		// e o defeito nasceu ali.
		//
		// Deixar o compare A disparar durante o planejamento e inofensivo: ele
		// so cumpre um corte que ja estava vencido. O que nao pode e a ISR dele
		// reentrar no planejamento, e disso cuida agendador_em_execucao.
		TIMSK1 &= ~(1 << OCIE1B);
		agendador_em_execucao = true;
		SREG = sreg; // devolve a interrupcao do dente imediatamente
	}

	// Origem angular congelada: sem o cli() global um gap novo pode chegar no
	// meio deste calculo e mudar tick_base_sincronismo, misturando duas
	// referencias entre os canais. O loop reprocessa a volta nova de qualquer
	// forma, porque a ISR remarca agendamento_pendente.
	PULSO_FASE_ALTO();
	const uint32_t base = tick_base_sincronismo;

	bool algo_desligou = processar_cortes_vencidos(base);
	limpar_ignicoes_pendentes_nao_acionadas();

	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (!ignicao_agendada[i] && !ign_acionado[i] && !captura_dwell[i]) {
			agendar_ignicao_canal(i, base);
		}
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		if (!inj_acionado[i] && !captura_req_fuel[i] && !injecao_agendada[i]) {
			agendar_injecao_canal(i, base);
		}
	}

	// As duas chamadas, sempre - exatamente como fazem as duas ISRs do Timer1.
	//
	// Antes a segunda era condicionada a algo_desligou, sob a ideia de que o
	// compare A so mudaria se processar_cortes_vencidos tivesse desligado algo.
	// A premissa e falsa: atualizar_compare_b_ligar() tem o caminho de
	// prevencao de deadlock, que LIGA a bobina inline quando o evento nasce
	// vencido - e isso cria um desligamento pendente novo, depois de
	// algo_desligou ja ter sido calculado la em cima.
	//
	// Com o compare A desarmado, ninguem arma o desligamento dessa bobina. E
	// como e a propria ISR do compare A que se rearmaria, ela nunca dispara: o
	// canal fica travado ligado ate protege_dwell_maximo() cortar, 1,5x o dwell
	// depois. Ou seja, a bobina carrega quase o dobro do previsto e a centelha
	// sai onde a protecao calhou de rodar, nao onde a tabela mandou.
	//
	// Atinge so o primeiro canal depois do gap (ign1 na 60-2 de 6 cilindros),
	// que e o unico com prazo curto o bastante para o evento nascer vencido.
	// Medido em bancada em rotacao fixa, 3239 ciclos por canal:
	//
	//   ign1: 28 dwells (0,86%) entre 4,05 e 4,50ms, contra 2,49ms de mediana
	//   ign2 e ign3: nenhum, maximo 2,52ms
	//
	// Todos os 28 acima de 1,5x o dwell, com excesso de 0,31 a 0,77ms - a
	// latencia do loop -, e nada entre 2,53 e 4,05ms. Mecanismo distinto, nao
	// cauda de distribuicao.
	// O ARMAMENTO final e atomico; o calculo pesado acima nao e.
	//
	// Com o OCIE1A desmascarado (ver acima), a ISR do compare A pode disparar no
	// meio destas duas chamadas - e ela chama exatamente as mesmas funcoes. Uma
	// desarma depois que a outra armou, e o canal fica sem desligamento.
	//
	// Fechar so este trecho custa pouco: sao duas varreduras de 3 canais, nao o
	// planejamento inteiro. A interrupcao do dente, que foi o motivo de estreitar
	// a secao critica, continua livre durante todo o calculo.
	PULSO_FASE_BAIXO();   // fim da fase 1: agendamento dos canais
	PULSO_FASE_ALTO();    // inicio da fase 2: atualizacao dos comparadores
	uint8_t sreg_arme = SREG;
	cli();
	atualizar_compare_b_ligar();
	atualizar_compare_a_desligar();
	agendador_em_execucao = false;
	SREG = sreg_arme;
	PULSO_FASE_BAIXO();
	(void)algo_desligou;
	if (proteger_contra_isr_do_dente) {
		SREG = sreg;
	} else {
		// NAO restaurar TIMSK1 inteiro aqui. As duas funcoes acima gerenciam o
		// proprio bit: armam quando ha evento e chamam desabilitar_timer1_*
		// quando nao ha, ou seja o estado que elas deixam JA e o correto.
		//
		// Restaurar o valor salvo apagava o arme que atualizar_compare_b_ligar
		// acabara de fazer, e o resultado em bancada foi ignicao morta de 1200
		// a 3500 rpm nos tres canais. Acima disso a ign1 reaparecia
		// gradualmente (40% em 3500, 100% em 4500) porque com os eventos mais
		// proximos entra o caminho de prevencao de deadlock, que aciona a
		// bobina direto sem depender da interrupcao - o que mascarava a causa.
		//
		// Nem OCIE1B nem OCIE1A precisam de restauracao: atualizar_compare_b_ligar
		// e atualizar_compare_a_desligar rodam as duas, sempre, e cada uma deixa
		// o proprio bit no estado certo. Restaurar o valor salvo aqui apagaria o
		// arme que elas acabaram de fazer.
		//
		// Havia um restore condicional de OCIE1A para o caso de
		// atualizar_compare_a_desligar nao rodar. Ele deixou de existir junto
		// com a condicao: com a chamada incondicional, restaurar seria o erro.
		(void)timsk_salvo;
		SREG = sreg;
	}
}


ISR(TIMER1_OVF_vect) {
	timer1_overflow_count++;
}

// As duas ISRs compartilham o mesmo pino de medicao: o que se quer saber e o
// tempo TOTAL em que o Timer1 bloqueia o resto do sistema, nao qual das duas
// gastou. Elas nunca se sobrepoem (interrupcao nao aninha no AVR), entao cada
// pulso corresponde a uma execucao - ou a duas coladas, se uma disparar logo
// apos a outra, o que conta como bloqueio continuo do mesmo jeito.
ISR(TIMER1_COMPB_vect) {
	PULSO_TIMER1_ALTO();
	uint32_t tick_atual = ler_tick32_timer1();
	processar_ligamentos_vencidos(tick_atual);
	processar_cortes_vencidos(tick_atual);

	atualizar_compare_b_ligar();
	atualizar_compare_a_desligar();
	PULSO_TIMER1_BAIXO();
}

ISR(TIMER1_COMPA_vect) {
	PULSO_TIMER1_ALTO();
	uint32_t tick_atual = ler_tick32_timer1();
	processar_cortes_vencidos(tick_atual);

	// Com o agendador em curso, faz so o desligamento e o proprio rearme. O
	// planejamento de quem LIGA e do agendador, que o refaz no fim - reentrar
	// nele daqui seria mexer no mesmo estado que ele esta montando.
	if (agendador_em_execucao) {
		atualizar_compare_a_desligar();
		PULSO_TIMER1_BAIXO();
		return;
	}

	// DESLIGAR antes, LIGAR depois. Parece invertido em relacao a ISR do
	// compare B, e por raciocinio eu troquei a ordem aqui - "ligar cria
	// desligamento, entao armar o desligar por ultimo". A bancada reprovou:
	// com B antes de A, os dwells presos a 4000rpm passaram de 0,14% para 7,4%
	// das voltas. Revertido.
	//
	// Nao tenho explicacao medida para o porque, e por isso a ordem fica como
	// estava e este comentario fica aqui: e um ponto sensivel que ja custou uma
	// regressao, nao um detalhe de estilo. Quem for mexer, meça antes.
	atualizar_compare_a_desligar();
	atualizar_compare_b_ligar();
	PULSO_TIMER1_BAIXO();
}
