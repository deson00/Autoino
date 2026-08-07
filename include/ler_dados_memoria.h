// Esta função envia os dados via serial no formato esperado

// Passa cada byte pelo mesmo CRC8 (polinomio 0x07) ja usado no quadro de
// telemetria, e so escreve na serial de verdade se nao estiver no modo
// "somente CRC" (comando 'x') - assim o comando 'x' reusa 100% da logica
// de serializacao de ler_dados_memoria() sem gastar flash com uma copia
// dela, e sem precisar mandar o dump inteiro so pra saber se mudou algo.
void enviar_byte_config(byte valor) {
  crc_config_atual ^= valor;
  for (byte bit = 0; bit < 8; bit++) {
    crc_config_atual = (crc_config_atual & 0x80U) ? (byte)((crc_config_atual << 1) ^ 0x07U) : (byte)(crc_config_atual << 1);
  }
  if (!modo_somente_crc_config) {
    Serial.write(valor);
  }
}

void sendSerialString(const char* str) {
  while (*str) {
    enviar_byte_config(*str++);
  }
}

void sendSerialInt(int value) {
  char buffer[10]; // Buffer para converter int para string
  itoa(value, buffer, 10); // Converter int para string
  sendSerialString(buffer); // Enviar a string resultante
}

void ler_dados_memoria() {
  // ========== PRIMEIRO: LER TODOS OS DADOS DA EEPROM ==========
  ler_dados_eeprom(); // Esta chamada estava faltando!
  // ========== TABELA IGNIÇÃO
  // Enviar o identificador de comando
  // a) Vetor MAP/TPS ignição
  enviar_byte_config(';');
  enviar_byte_config('a');
  enviar_byte_config(',');

  // Enviar os valores do vetor_map_tps
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_map_tps[i]);

      enviar_byte_config(',');

  }
  enviar_byte_config(';');
  // b) Vetor RPM ignição
  enviar_byte_config('b');
  enviar_byte_config(',');
  // vetor rpm
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_rpm[i]);

      enviar_byte_config(',');

  }
  enviar_byte_config(';');

  // transforma matriz em vetor
  enviar_byte_config('c');
  enviar_byte_config(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      sendSerialInt(matriz_avanco[i][j]);

        enviar_byte_config(',');

    }
  }
  enviar_byte_config(';');
  // ========== CONFIGURAÇÕES SISTEMA ==========

  // g) Configuração inicial
  enviar_byte_config('g');
  enviar_byte_config(',');
  sendSerialInt(tipo_ignicao);
  enviar_byte_config(',');
  sendSerialInt(qtd_dente);
  enviar_byte_config(',');
  sendSerialInt(local_rodafonica);
  enviar_byte_config(',');
  sendSerialInt(qtd_dente_faltante);
  enviar_byte_config(',');
  sendSerialInt(grau_pms);
  enviar_byte_config(',');
  sendSerialInt(qtd_cilindro); // Removida a multiplicação corrompida
  enviar_byte_config(',');
  enviar_byte_config(';');

  // j) Configuração faisca
  enviar_byte_config('j');
  enviar_byte_config(',');
  sendSerialInt(referencia_leitura_ignicao);
  enviar_byte_config(',');
  sendSerialInt(modo_ignicao);
  enviar_byte_config(',');
  sendSerialInt(grau_avanco_partida);
  enviar_byte_config(',');
  sendSerialInt(avanco_fixo);
  enviar_byte_config(',');
  sendSerialInt(grau_avanco_fixo);
  enviar_byte_config(',');
  sendSerialInt(tipo_sinal_bobina);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // k) Configuração dwell
  enviar_byte_config('k');
  enviar_byte_config(',');
  sendSerialInt(dwell_partida);
  enviar_byte_config(',');
  sendSerialInt(dwell_funcionamento);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // l) Configuração sensor temperatura CLT
  enviar_byte_config('l');
  enviar_byte_config(','); // letra L minúsculo
  sendSerialInt(referencia_temperatura_clt1);
  enviar_byte_config(',');
  sendSerialInt(referencia_resistencia_clt1);
  enviar_byte_config(',');
  sendSerialInt(referencia_temperatura_clt2);
  enviar_byte_config(',');
  sendSerialInt(referencia_resistencia_clt2);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // u) Configuração sensor temperatura do ar IAT
  enviar_byte_config('u');
  enviar_byte_config(',');
  sendSerialInt(referencia_temperatura_iat1);
  enviar_byte_config(',');
  sendSerialInt(referencia_resistencia_iat1);
  enviar_byte_config(',');
  sendSerialInt(referencia_temperatura_iat2);
  enviar_byte_config(',');
  sendSerialInt(referencia_resistencia_iat2);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // ========== TABELA VE ==========

  // d) Vetor MAP/TPS VE
  enviar_byte_config('d');
  enviar_byte_config(',');
  // vetor map ou tps da ve
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_map_tps_ve[i]);

      enviar_byte_config(',');

  }
  enviar_byte_config(';');

  // e) Vetor RPM VE
  enviar_byte_config('e');
  enviar_byte_config(',');
  // vetor rpm da tabela ve
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_rpm_ve[i]);

      enviar_byte_config(',');

  }
  enviar_byte_config(';');

  // f) Matriz VE (como vetor linear)
  enviar_byte_config('f');
  enviar_byte_config(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      sendSerialInt(matriz_ve[i][j]);

        enviar_byte_config(',');

    }
  }
  enviar_byte_config(';');

  // ========== CONFIGURAÇÕES INJEÇÃO E PROTEÇÃO ==========

  // m) Configuração injeção
  enviar_byte_config('m');
  enviar_byte_config(',');
  sendSerialInt(referencia_leitura_injecao);
  enviar_byte_config(',');
  sendSerialInt(tipo_motor);
  enviar_byte_config(',');
  sendSerialInt(modo_injecao);
  enviar_byte_config(',');
  sendSerialInt(emparelhar_injetor);
  enviar_byte_config(',');
  sendSerialInt(deslocamento_motor);
  enviar_byte_config(',');
  sendSerialInt(numero_cilindro_injecao);
  enviar_byte_config(',');
  sendSerialInt(numero_injetor);
  enviar_byte_config(',');
  sendSerialInt(numero_esguicho);
  enviar_byte_config(',');
  sendSerialInt(tamanho_injetor);
  enviar_byte_config(',');
  sendSerialInt(tipo_acionamento_injetor);
  enviar_byte_config(',');
  sendSerialInt(tipo_combustivel);
  enviar_byte_config(',');
  sendSerialInt(REQ_FUEL);
  enviar_byte_config(',');
  sendSerialInt(dreq_fuel);
  enviar_byte_config(',');
  sendSerialInt(tipo_sonda_o2);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // t) Parametros do injetor
  enviar_byte_config('t');
  enviar_byte_config(',');
  sendSerialInt(limite_injetor);
  enviar_byte_config(',');
  sendSerialInt(tempo_abertura_injetor);
  enviar_byte_config(',');
  sendSerialInt(grau_fechamento_injetor);
  enviar_byte_config(',');
  sendSerialInt(acrescimo_injecao_partida);
  enviar_byte_config(',');
  sendSerialInt(acrescimo_injecao_funcionamento);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // n) Configuração proteção e limites
  enviar_byte_config('n');
  enviar_byte_config(',');
  sendSerialInt(tipo_protecao);
  enviar_byte_config(',');
  sendSerialInt(rpm_pre_corte);
  enviar_byte_config(',');
  sendSerialInt(avanco_corte);
  enviar_byte_config(',');
  sendSerialInt(tempo_corte);
  enviar_byte_config(',');
  sendSerialInt(rpm_maximo_corte);
  enviar_byte_config(',');
  sendSerialInt(numero_base_corte);
  enviar_byte_config(',');
  sendSerialInt(qtd_corte);
  enviar_byte_config(',');
  enviar_byte_config(';');

    // o) Enriquecimento na aceleração
    enviar_byte_config('o');
    enviar_byte_config(',');
    sendSerialInt(enriquecimento_aceleracao[0]);
    enviar_byte_config(',');
    sendSerialInt(enriquecimento_aceleracao[1]);
    enviar_byte_config(',');
    sendSerialInt(enriquecimento_aceleracao[2]);
    enviar_byte_config(',');
    sendSerialInt(enriquecimento_aceleracao[3]);
    enviar_byte_config(',');
    sendSerialInt(enriquecimento_aceleracao[4]);
    enviar_byte_config(',');
    sendSerialInt(tps_dot_escala[0]);
    enviar_byte_config(',');
    sendSerialInt(tps_dot_escala[1]);
    enviar_byte_config(',');
    sendSerialInt(tps_dot_escala[2]);
    enviar_byte_config(',');
    sendSerialInt(tps_dot_escala[3]);
    enviar_byte_config(',');
    sendSerialInt(tps_dot_escala[4]);
    enviar_byte_config(',');
    sendSerialInt(tipo_verificacao_aceleracao_rapida);
    enviar_byte_config(',');
    sendSerialInt(tps_mudanca_minima);
    enviar_byte_config(',');
    sendSerialInt(intervalo_tempo_aceleracao);
    enviar_byte_config(',');
    sendSerialInt(duracao_enriquecimento);
    enviar_byte_config(',');
    sendSerialInt(rpm_minimo_enriquecimento);
    enviar_byte_config(',');
    sendSerialInt(rpm_maximo_enriquecimento);
    enviar_byte_config(',');
    sendSerialInt(enriquecimento_desaceleracao);
    enviar_byte_config(',');
    enviar_byte_config(';');

  // p) Configuração TPS
  enviar_byte_config('p');
  enviar_byte_config(',');
  sendSerialInt(valor_tps_minimo);
  enviar_byte_config(',');
  sendSerialInt(valor_tps_maximo);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // q) Configuração MAP
    enviar_byte_config('q');
    enviar_byte_config(',');
    sendSerialInt(valor_map_tipo);
    enviar_byte_config(',');
    sendSerialInt(valor_map_minimo);
    enviar_byte_config(',');
    sendSerialInt(valor_map_maximo);
    enviar_byte_config(',');
    enviar_byte_config(';');

  // r) Enriquecimento de injeção por temperatura (5 pontos)
  enviar_byte_config('r');
  enviar_byte_config(',');
  for (int i = 0; i < 5; i++) {
    sendSerialInt(vetor_temperatura_injecao[i]);
    enviar_byte_config(',');
  }
  for (int i = 0; i < 5; i++) {
    sendSerialInt(vetor_enriquecimento_temperatura[i]);
    enviar_byte_config(',');
  }
  sendSerialInt(usar_injecao_temperatura);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // s) Avanco por temperatura (5 pontos)
  enviar_byte_config('s');
  enviar_byte_config(',');
  for (int i = 0; i < 5; i++) {
    sendSerialInt(vetor_temperatura[i]);
    enviar_byte_config(',');
  }
  for (int i = 0; i < 5; i++) {
    sendSerialInt(vetor_avanco_temperatura[i]);
    enviar_byte_config(',');
  }
  sendSerialInt(usar_avanco_temperatura);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // v) Configuracao de partida
  enviar_byte_config('v');
  enviar_byte_config(',');
  sendSerialInt(rpm_partida);
  enviar_byte_config(',');
  sendSerialInt(nivel_limpeza_afogamento);
  enviar_byte_config(',');
  sendSerialInt(atraso_injecao_inicial);
  enviar_byte_config(',');
  sendSerialInt(tempo_reducao_enriquecimento_partida);
  enviar_byte_config(',');
  enviar_byte_config(';');

  // w) Controle de marcha lenta
  enviar_byte_config('w');
  enviar_byte_config(',');
  sendSerialInt(modo_marcha_lenta);
  enviar_byte_config(',');
  sendSerialInt(temperatura_desligamento_marcha_lenta);
  enviar_byte_config(',');
  sendSerialInt(histerese_marcha_lenta);
  enviar_byte_config(',');
  sendSerialInt(pwm_marcha_lenta_frio);
  enviar_byte_config(',');
  sendSerialInt(pwm_marcha_lenta_quente);
  enviar_byte_config(',');
  sendSerialInt(rpm_alvo_marcha_lenta);
  enviar_byte_config(',');
  sendSerialInt(maximo_passos_marcha_lenta);
  enviar_byte_config(',');
  sendSerialInt(inverter_direcao_marcha_lenta);
  enviar_byte_config(',');
  enviar_byte_config(';');

}
