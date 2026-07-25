

enum FlagsEstadoMotor : byte {
  FLAG_MOTOR_SINCRONIZADO        = 1U << 0,
  FLAG_MOTOR_EM_PARTIDA          = 1U << 1,
  FLAG_MOTOR_FUNCIONANDO         = 1U << 2,
  FLAG_PROTECAO_ATIVA            = 1U << 3,
  FLAG_ENRIQUECIMENTO_ACELERACAO = 1U << 4,
  FLAG_LIMPEZA_AFOGAMENTO        = 1U << 5,
  FLAG_MARCHA_LENTA_ATIVA        = 1U << 6,
  FLAG_ENRIQUECIMENTO_PARTIDA    = 1U << 7
};

static inline byte montar_flags_estado_motor() {
  byte flags = 0;

  if (revolucoes_sincronizada >= 1) flags |= FLAG_MOTOR_SINCRONIZADO;
  if (rpm_anterior >= 20 && rpm_anterior < (int)rpm_partida) flags |= FLAG_MOTOR_EM_PARTIDA;
  if (rpm_anterior >= (int)rpm_partida) flags |= FLAG_MOTOR_FUNCIONANDO;
  if (status_corte != 0) flags |= FLAG_PROTECAO_ATIVA;
  if (incremento_aceleracao > 0) flags |= FLAG_ENRIQUECIMENTO_ACELERACAO;
  if (limpeza_afogamento_ativa) flags |= FLAG_LIMPEZA_AFOGAMENTO;
  if (saida_marcha_lenta_ativa) flags |= FLAG_MARCHA_LENTA_ATIVA;
  if (enriquecimento_partida_atual > 0) flags |= FLAG_ENRIQUECIMENTO_PARTIDA;

  return flags;
}
enum ProtocoloTempoReal : byte {
  TELEMETRIA_CABECALHO_1 = 0xA5,
  TELEMETRIA_CABECALHO_2 = 0x5A,
  TELEMETRIA_VERSAO = 2,
  TELEMETRIA_TAMANHO_QUADRO = 40,
  TELEMETRIA_TAMANHO_CABECALHO = 4,
  TELEMETRIA_TAMANHO_PAYLOAD = 35,
  TELEMETRIA_TAMANHO_CRC = 1,
  TELEMETRIA_EXTENSAO_MAGIC = 0xA2,
  TELEMETRIA_BYTES_RESERVADOS = TELEMETRIA_TAMANHO_QUADRO
      - TELEMETRIA_TAMANHO_CABECALHO
      - TELEMETRIA_TAMANHO_PAYLOAD
      - TELEMETRIA_TAMANHO_CRC
};

static inline byte obter_abertura_marcha_lenta_telemetria() {
  if (modo_marcha_lenta == 0) return 0;
  if (modo_marcha_lenta == 1) return saida_marcha_lenta_ativa ? 100 : 0;
  if (modo_marcha_lenta == 3) {
    if (maximo_passos_marcha_lenta == 0) return 0;
    return (byte)(((unsigned int)posicao_passo_marcha_lenta * 100U) /
                  (unsigned int)maximo_passos_marcha_lenta);
  }
  return abertura_marcha_lenta_atual;
}

static inline byte calcular_duty_cycle_telemetria() {
  if (rpm_anterior <= 0 || numero_esguicho <= 0) return 0;

  unsigned long duracao_ciclo_us = tipo_motor == 2
      ? 60000000UL / (unsigned long)rpm_anterior
      : 120000000UL / (unsigned long)rpm_anterior;
  unsigned long pulso_us = min(tempo_injecao, 65535UL);
  unsigned int esguichos = (unsigned int)constrain(numero_esguicho, 1, 20);
  unsigned long tempo_acionado_us = pulso_us * esguichos;
  unsigned long duty = duracao_ciclo_us > 0
      ? (tempo_acionado_us * 100UL) / duracao_ciclo_us
      : 0;
  return (byte)min(duty, 100UL);
}

static inline byte atualizar_crc8_telemetria(byte crc, byte valor) {
  crc ^= valor;
  for (byte bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80U) ? (byte)((crc << 1) ^ 0x07U) : (byte)(crc << 1);
  }
  return crc;
}

static inline void enviar_u8_telemetria(byte valor, byte &crc) {
  Serial.write(valor);
  crc = atualizar_crc8_telemetria(crc, valor);
}

static inline void enviar_u16_telemetria(uint16_t valor, byte &crc) {
  enviar_u8_telemetria((byte)(valor & 0xFFU), crc);
  enviar_u8_telemetria((byte)((valor >> 8) & 0xFFU), crc);
}

void envia_dados_tempo_real(int indice_envio){
  if (!status_dados_tempo_real || indice_envio != 1) {
    return;
  }

  byte crc = 0;

  enviar_u8_telemetria(TELEMETRIA_CABECALHO_1, crc);
  enviar_u8_telemetria(TELEMETRIA_CABECALHO_2, crc);
  enviar_u8_telemetria(TELEMETRIA_VERSAO, crc);
  enviar_u8_telemetria(TELEMETRIA_TAMANHO_QUADRO, crc);

  enviar_u16_telemetria((uint16_t)rpm_anterior, crc);
  enviar_u16_telemetria((uint16_t)valor_map, crc);
  enviar_u8_telemetria(temperatura_motor, crc);
  enviar_u8_telemetria(grau_avanco, crc);
  enviar_u16_telemetria((uint16_t)(qtd_loop * 5), crc);
  enviar_u8_telemetria(qtd_perda_sincronia, crc);
  enviar_u8_telemetria((byte)VE, crc);
  enviar_u16_telemetria((uint16_t)tempo_injecao, crc);
  enviar_u8_telemetria((byte)valor_tps, crc);
  enviar_u8_telemetria((byte)status_corte, crc);
  enviar_u16_telemetria((uint16_t)incremento_aceleracao, crc);
  enviar_u16_telemetria(valor_o2_adc, crc);
  enviar_u16_telemetria((uint16_t)valor_tps_adc, crc);
  enviar_u8_telemetria(montar_flags_estado_motor(), crc);
  enviar_u16_telemetria((uint16_t)analogRead(pino_sensor_flex), crc);
  enviar_u16_telemetria((uint16_t)analogRead(pino_sensor_pressao_oleo), crc);
  enviar_u8_telemetria(temperatura_ar, crc);
  enviar_u16_telemetria((uint16_t)analogRead(pino_sensor_brv), crc);
  enviar_u16_telemetria((uint16_t)min(dwell_bobina, 65535UL), crc);
  enviar_u8_telemetria(obter_abertura_marcha_lenta_telemetria(), crc);
  enviar_u16_telemetria((uint16_t)rpm_alvo_marcha_lenta, crc);
  enviar_u8_telemetria(calcular_duty_cycle_telemetria(), crc);
  enviar_u8_telemetria(TELEMETRIA_EXTENSAO_MAGIC, crc);

  for (byte i = 0; i < TELEMETRIA_BYTES_RESERVADOS; i++) {
    enviar_u8_telemetria(0, crc);
  }

  Serial.write(crc);
}
