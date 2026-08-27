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

static inline void desabilitar_timer1_compare_a() {
	TIMSK1 &= ~(1 << OCIE1A);
}

static inline void desabilitar_timer1_compare_b() {
	TIMSK1 &= ~(1 << OCIE1B);
}

static void atualizar_compare_b_ligar();
static void atualizar_compare_a_desligar();
static inline void agendar_injecao_canal(int i, uint32_t tick_atual);
static inline bool processar_cortes_vencidos(uint32_t tick_atual);

// Limpa apenas agendamentos VENCIDOS (a hora de ligar ja passou e o canal nao
// ligou). Antes limpava todo agendamento pendente a cada gap, o que impedia
// que um dwell atravessasse o gap: um canal cujo carregamento precisa comecar
// no fim de uma volta para a centelha sair no comeco da seguinte era apagado
// justo no gap entre as duas, e nunca disparava. Canais que ja dispararam sao
// desarmados por processar_cortes_vencidos, entao continuam sendo reagendados
// normalmente todo ciclo - o que sobrevive aqui e so o agendamento ainda
// valido, apontando para o futuro.
static inline void limpar_ignicoes_pendentes_nao_acionadas(uint32_t tick_agora) {
	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (ignicao_agendada[i] && !ign_acionado[i] && !captura_dwell[i] &&
		    tick_ja_passou(tick_agora, ignicao_tick_ligar[i])) {
			ignicao_agendada[i] = false;
		}
	}
}

static inline uint32_t calcular_tick_fim_dwell_futuro(unsigned long tempo_ignicao_us, uint32_t tick_atual, uint32_t dwell_ticks) {
	// tick_base_sincronismo e a ORIGEM ANGULAR (instante do gap); o tick lido
	// aqui e o "agora" real, porque esta funcao roda no loop() com atraso
	// variavel depois do gap.
	//
	// Uma tentativa anterior de usar o tempo real aqui piorou tudo (cobertura
	// caiu para 50% e o firmware morria em 4600rpm). Aquilo era efeito do bug
	// de cancelamento cruzado, ja corrigido: usar o tempo real fazia mais
	// canais caírem no caminho de "dwell curto", e naquela epoca cada canal
	// que caia apagava o agendamento de TODOS. Com o cancelamento por canal, a
	// premissa volta a valer.
	uint32_t tick_fim_dwell = tick_base_sincronismo + us_para_ticks_timer1(tempo_ignicao_us);

	if (tempo_cada_grau > 0) {
		uint32_t periodo_ticks_360 = us_para_ticks_timer1(360UL * tempo_cada_grau);
		uint32_t agora = ler_tick32_timer1();

		// Se o dwell nao couber inteiro antes do alvo desta volta, joga a
		// centelha para a volta SEGUINTE: o carregamento passa a comecar perto
		// do fim desta volta e atravessa o gap, em vez de ser cancelado.
		//
		// E o caso do canal cujo angulo de centelha fica logo apos o gap (com 6
		// cilindros em centelha perdida, 93 graus): a folga dele e
		// volta*93/360 menos o dwell, que a 5000rpm ja e de 0,1ms e acima de
		// ~5200rpm fica NEGATIVA - o dwell simplesmente nao cabe ali. Comparacao
		// com sinal porque o alvo pode estar no passado.
		int32_t ticks_ate_inicio = (int32_t)(tick_fim_dwell - dwell_ticks - agora);
		if (ticks_ate_inicio < (int32_t)TIMER1_MIN_DELTA_TICKS) {
			tick_fim_dwell += periodo_ticks_360;
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

	uint32_t dwell_ticks = us_para_ticks_timer1(dwell_bobina);
	calcula_grau_ignicao(i);
	uint32_t tick_fim_dwell = calcular_tick_fim_dwell_futuro(tempo_proxima_ignicao[i], tick_atual, dwell_ticks);

	uint32_t tick_inicio_dwell = tick_fim_dwell - dwell_ticks;

	ignicao_tick_ligar[i] = tick_inicio_dwell;
	ignicao_tick_desligar[i] = tick_fim_dwell;
	ignicao_agendada[i] = true;
}

static inline void recalcular_ignicao_canal_por_dente(int i, uint32_t tick_atual) {
	if (status_corte != 0 || tempo_cada_grau == 0 || grau_cada_dente == 0) {
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
	atualizar_compare_b_ligar();
	// So precisa reprogramar o "desligar" se algo realmente desligou acima
	// (processar_cortes_vencidos) - o recalculo por dente so mexe em canais
	// que ainda nao ligaram, entao nao afeta o conjunto de quem esta ligado.
	if (algo_desligou) {
		atualizar_compare_a_desligar();
	}

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
		uint32_t periodo_ticks_360 = us_para_ticks_timer1(360UL * tempo_cada_grau);

		if (tick_inicio_injecao == tick_atual) {
			tick_inicio_injecao += TIMER1_MIN_DELTA_TICKS;
		}

		tick_inicio_injecao = alinhar_tick_para_futuro(tick_inicio_injecao, tick_atual, periodo_ticks_360);
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

	uint32_t periodo_ticks_360 = us_para_ticks_timer1(360UL * tempo_cada_grau);
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
	uint32_t dwell_minimo_util_ticks = (dwell_ticks * 80UL) / 100UL;
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
		if (ignicao_agendada[i] && ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_desligar[i])) {
			desligar_dwell(i);
			ignicao_agendada[i] = false;
			algo_desligou = true;
			// Rearma o canal assim que a centelha sai, em vez de esperar o
			// proximo gap. Antes so a roda no comando (local_rodafonica == 1)
			// fazia isso; no virabrequim o canal ficava sem agendamento ate o
			// gap seguinte. Com o dwell podendo atravessar a volta, isso criava
			// um teto de 50%: o agendamento ocupava duas voltas (agenda na
			// volta N, dispara na N+1) e o canal so podia ser reagendado na
			// N+2, porque cada canal tem um unico slot. Medido em ~48% no canal
			// de 93 graus. Rearmando aqui, o slot volta a ficar livre no
			// instante do disparo e o alvo da volta seguinte e calculado
			// corretamente pela logica de cruzar a volta.
			if (status_corte == 0) {
				agendar_ignicao_canal(i, tick_atual);
			}
		}
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		if (injecao_agendada[i] && inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_desligar[i])) {
			desligar_injetor(i);
			injecao_agendada[i] = false;
			algo_desligou = true;
			if (local_rodafonica == 1) {
				agendar_injecao_canal(i, tick_atual);
			}
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

	for (int i = 0; i < numero_injetor; i++) {
		digitalWrite(injecao_pins[i], LOW);
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
			desabilitar_timer1_compare_a();
			break;
		}

		uint32_t agora = ler_tick32_timer1();
		bool encontrado = false;
		uint32_t menor_delta = 0xFFFFFFFFUL;
		uint32_t proximo_tick = 0;

		byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
		for (int i = 0; i < eventos_ignicao; i++) {
			if (ignicao_agendada[i] && ign_acionado[i]) {
				uint32_t alvo = ignicao_tick_desligar[i];
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
			if (injecao_agendada[i] && inj_acionado[i]) {
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

	// tick_base_sincronismo NAO e lido aqui - vem ja capturado pela interrupcao
	// no proprio instante do dente de falha (ver decoder.h), pra nao perder
	// precisao caso essa funcao rode com atraso a partir do loop().
	//
	// Protecao restrita ao Timer1 (OCIE1A/OCIE1B), NAO cli() global: quem
	// disputa esses arrays e os registradores OCR1A/OCR1B e a propria ISR de
	// comparacao do Timer1 (TIMER1_COMPA/COMPB_vect), nao a interrupcao do
	// sensor de rotacao (pino externo, INT0/INT1 - registrador EIMSK, totalmente
	// separado do TIMSK1). Usar cli() global aqui bloqueava tambem o dente
	// pela duracao inteira desse calculo pesado, e como ele agora roda com
	// atraso variavel a partir do loop(), em RPM alto (dente a cada ~150-300us)
	// isso perdia dentes de verdade - corrompendo o sincronismo por tras do
	// sinal (visivel so como falha silenciosa de agendamento, nao no log bruto).
	TIMSK1 &= ~((1 << OCIE1A) | (1 << OCIE1B));

	processar_cortes_vencidos(tick_base_sincronismo);
	limpar_ignicoes_pendentes_nao_acionadas(ler_tick32_timer1());

	byte eventos_ignicao = quantidade_eventos_ignicao_por_ciclo_sensor();
	for (int i = 0; i < eventos_ignicao; i++) {
		if (!ignicao_agendada[i] && !ign_acionado[i] && !captura_dwell[i]) {
			agendar_ignicao_canal(i, tick_base_sincronismo);
		}
	}

	byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
	for (int i = 0; i < eventos_injecao; i++) {
		if (!inj_acionado[i] && !captura_req_fuel[i] && !injecao_agendada[i]) {
			agendar_injecao_canal(i, tick_base_sincronismo);
		}
	}

	// Fase 2 (programar o hardware): TESTE - sem cli() global aqui.
	//
	// O risco de tirar e reentrancia: atualizar_compare_b_ligar re-arma OCIE1B
	// no meio dela mesma, entao a partir dali um match real pode disparar
	// TIMER1_COMPB_vect por cima destas duas funcoes, que nao sao reentrantes.
	// O pior caso concreto e o ramo !encontrado chamar desabilitar_timer1_compare_*
	// logo depois da ISR ter armado aquele comparador - canal orfao, bobina
	// ligada sem desligamento agendado. Isso e coberto pelo
	// processar_cortes_vencidos da volta seguinte e por protege_dwell_maximo (1.5x).
	//
	// Em troca, some mais uma janela de bloqueio do dente em posicao ALEATORIA
	// da volta - que e o que quebra a contagem do decoder (ver a validacao de
	// posicao do gap em decoder.h). Um bloqueio maior que meio periodo de dente
	// vira gap falso; esse limiar cai junto com o periodo, por isso o problema
	// aparecia a partir de ~2000rpm e piorava dai pra cima.
	atualizar_compare_b_ligar();
	atualizar_compare_a_desligar();
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

	atualizar_compare_a_desligar();
	atualizar_compare_b_ligar();
	PULSO_TIMER1_BAIXO();
}
