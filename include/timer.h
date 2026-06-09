static const uint16_t TIMER1_TICK_US = 4;
static const uint16_t TIMER1_MIN_DELTA_TICKS = 2;
static const uint8_t TIMER1_MAX_REPLAN_LOOPS = 32;

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

static inline unsigned long ajustar_tempo_evento_borda_referencia(unsigned long tempo_evento_us) {
	if (tempo_cada_grau > 0) {
		unsigned long periodo_referencia_us = 360UL * tempo_cada_grau;
		if (periodo_referencia_us > 0 && (tempo_evento_us % periodo_referencia_us) == 0) {
			if (tempo_evento_us >= tempo_cada_grau) {
				tempo_evento_us -= tempo_cada_grau;
			} else {
				tempo_evento_us += tempo_cada_grau;
			}
		}
	}
	return tempo_evento_us;
}

static inline void desabilitar_timer1_compare_a() {
	TIMSK1 &= ~(1 << OCIE1A);
}

static inline void desabilitar_timer1_compare_b() {
	TIMSK1 &= ~(1 << OCIE1B);
}

static void atualizar_compare_b_ligar();
static void atualizar_compare_a_desligar();

static inline uint32_t calcular_tick_fim_dwell_futuro(unsigned long tempo_ignicao_us, uint32_t tick_atual, uint32_t dwell_ticks) {
	uint32_t tick_fim_dwell = tick_base_sincronismo + us_para_ticks_timer1(tempo_ignicao_us);

	if (tempo_cada_grau > 0) {
		uint32_t periodo_ticks_360 = us_para_ticks_timer1(360UL * tempo_cada_grau);
		
		if (tick_fim_dwell == tick_atual) {
			tick_fim_dwell += TIMER1_MIN_DELTA_TICKS;
		}

		tick_fim_dwell = alinhar_tick_para_futuro_com_margem(tick_fim_dwell, tick_atual, periodo_ticks_360, dwell_ticks + TIMER1_MIN_DELTA_TICKS);
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

static inline void processar_ligamentos_vencidos(uint32_t tick_atual) {
	for (int i = 0; i < qtd_cilindro; i++) {
		if (ignicao_agendada[i] && !ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_ligar[i])) {
			iniciar_dwell(i);
		}

		if (injecao_agendada[i] && !inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_ligar[i])) {
			ligar_injetor(i);
		}
	}
}

static inline void limpar_ligamentos_vencidos_sem_acionar(uint32_t tick_atual) {
	for (int i = 0; i < qtd_cilindro; i++) {
		if (ignicao_agendada[i] && !ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_ligar[i])) {
			ignicao_agendada[i] = false;
		}

		if (injecao_agendada[i] && !inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_ligar[i])) {
			injecao_agendada[i] = false;
		}
	}
}

static inline void adiar_desligamento_ignicao_por_referencia(int i, uint32_t tick_atual) {
	uint32_t atraso_ticks = TIMER1_MIN_DELTA_TICKS;
	if (tempo_cada_grau > 0) {
		atraso_ticks = us_para_ticks_timer1(tempo_cada_grau);
	}
	if (atraso_ticks < 20) {
		atraso_ticks = 20;
	}
	ignicao_tick_desligar[i] = tick_atual + atraso_ticks;
}

static inline void processar_cortes_vencidos(uint32_t tick_atual) {
	for (int i = 0; i < qtd_cilindro; i++) {
		if (ignicao_agendada[i] && ign_acionado[i] && tick_ja_passou(tick_atual, ignicao_tick_desligar[i])) {
			if (!referencia_ignicao_valida_baixa_rotacao(i)) {
				adiar_desligamento_ignicao_por_referencia(i, tick_atual);
				continue;
			}
			desligar_dwell(i);
			ignicao_agendada[i] = false;
			// Re-agendar imediatamente se perdeu o ciclo do sincronismo
			if (local_rodafonica == 2 && revolucoes_sincronizada >= 1) {
				agendar_ignicao_canal(i, tick_atual);
			}
		}

		if (injecao_agendada[i] && inj_acionado[i] && tick_ja_passou(tick_atual, injecao_tick_desligar[i])) {
			desligar_injetor(i);
			injecao_agendada[i] = false;
			// Re-agendar imediatamente se perdeu o ciclo do sincronismo
			if (local_rodafonica == 2 && revolucoes_sincronizada >= 1) {
				agendar_injecao_canal(i, tick_atual);
			}
		}
	}
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
	for (int i = 0; i < qtd_cilindro; i++) {
		byte pino = ignicao_pins[(local_rodafonica == 1 && i >= (qtd_cilindro / 2)) ? (i - (qtd_cilindro / 2)) : i];
		digitalWrite(pino, LOW);
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

		for (int i = 0; i < qtd_cilindro; i++) {
			if (ignicao_agendada[i] && !ign_acionado[i]) {
				uint32_t alvo = ignicao_tick_ligar[i];
				uint32_t delta = delta_tick_evento(agora, alvo);
				if (!encontrado || delta < menor_delta) {
					encontrado = true;
					menor_delta = delta;
					proximo_tick = alvo;
				}
			}

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
		if (menor_delta < 20) {
			processar_ligamentos_vencidos(agora + menor_delta);
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

		for (int i = 0; i < qtd_cilindro; i++) {
			if (ignicao_agendada[i] && ign_acionado[i]) {
				uint32_t alvo = ignicao_tick_desligar[i];
				uint32_t delta = delta_tick_evento(agora, alvo);
				if (!encontrado || delta < menor_delta) {
					encontrado = true;
					menor_delta = delta;
					proximo_tick = alvo;
				}
			}

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
		if (menor_delta < 20) {
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

	uint8_t sreg = SREG;
	cli();
	tick_base_sincronismo = ler_tick32_timer1();
	processar_cortes_vencidos(tick_base_sincronismo);

	for (int i = 0; i < qtd_cilindro; i++) {
		if (!ignicao_agendada[i] && !ign_acionado[i] && !captura_dwell[i]) {
			agendar_ignicao_canal(i, tick_base_sincronismo);
		}

		if (!inj_acionado[i] && !captura_req_fuel[i] && !injecao_agendada[i]) {
			agendar_injecao_canal(i, tick_base_sincronismo);
		}
	}

	atualizar_compare_b_ligar();
	atualizar_compare_a_desligar();
	SREG = sreg;
}

ISR(TIMER1_OVF_vect) {
	timer1_overflow_count++;
}

ISR(TIMER1_COMPB_vect) {
	uint32_t tick_atual = ler_tick32_timer1();
	processar_ligamentos_vencidos(tick_atual);
	processar_cortes_vencidos(tick_atual);

	atualizar_compare_b_ligar();
	atualizar_compare_a_desligar();
}

ISR(TIMER1_COMPA_vect) {
	uint32_t tick_atual = ler_tick32_timer1();
	processar_cortes_vencidos(tick_atual);

	atualizar_compare_a_desligar();
	atualizar_compare_b_ligar();
}
