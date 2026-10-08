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

// Valor seguido de virgula, que e o par mais repetido deste dump (94 vezes).
// Uma chamada no lugar de duas em cada ponto devolve flash suficiente para a
// guarda angular da carga da bobina. Os bytes que saem na serial sao
// exatamente os mesmos, entao o protocolo com a UI nao muda. noinline porque o
// LTO, inlinando, copiaria o corpo de volta em cada ponto e anularia o ganho.
void __attribute__((noinline)) enviar_campo(int value) {
  sendSerialInt(value);
  enviar_byte_config(',');
}

void ler_dados_memoria() {
#ifndef DIAG_SEM_DUMP  // so para diagnostico que nao cabe na Nano com o dump

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
    enviar_campo(vetor_map_tps[i]);

  }
  enviar_byte_config(';');
  // b) Vetor RPM ignição
  enviar_byte_config('b');
  enviar_byte_config(',');
  // vetor rpm
  for (int i = 0; i < 16; i++) {
    enviar_campo(vetor_rpm[i]);

  }
  enviar_byte_config(';');

  // transforma matriz em vetor
  enviar_byte_config('c');
  enviar_byte_config(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      enviar_campo(matriz_avanco[i][j]);

    }
  }
  enviar_byte_config(';');
  // ========== CONFIGURAÇÕES SISTEMA ==========

  // g) Configuração inicial
  enviar_byte_config('g');
  enviar_byte_config(',');
  enviar_campo(tipo_ignicao);
  enviar_campo(qtd_dente);
  enviar_campo(local_rodafonica);
  enviar_campo(qtd_dente_faltante);
  enviar_campo(grau_pms);
  sendSerialInt(qtd_cilindro); // Removida a multiplicação corrompida
  enviar_byte_config(',');
  enviar_byte_config(';');

  // j) Configuração faisca
  enviar_byte_config('j');
  enviar_byte_config(',');
  enviar_campo(referencia_leitura_ignicao);
  enviar_campo(modo_ignicao);
  enviar_campo(grau_avanco_partida);
  enviar_campo(avanco_fixo);
  enviar_campo(grau_avanco_fixo);
  enviar_campo(tipo_sinal_bobina);
  enviar_byte_config(';');

  // k) Configuração dwell
  enviar_byte_config('k');
  enviar_byte_config(',');
  sendSerialInt(dwell_partida_us); // em us; a tela divide por 1000 pra mostrar em ms
  enviar_byte_config(',');
  enviar_campo(dwell_funcionamento_us);
  enviar_byte_config(';');

  // l) Configuração sensor temperatura CLT
  enviar_byte_config('l');
  enviar_byte_config(','); // letra L minúsculo
  enviar_campo(referencia_temperatura_clt1);
  enviar_campo(referencia_resistencia_clt1);
  enviar_campo(referencia_temperatura_clt2);
  enviar_campo(referencia_resistencia_clt2);
  enviar_byte_config(';');

  // u) Configuração sensor temperatura do ar IAT
  enviar_byte_config('u');
  enviar_byte_config(',');
  enviar_campo(referencia_temperatura_iat1);
  enviar_campo(referencia_resistencia_iat1);
  enviar_campo(referencia_temperatura_iat2);
  enviar_campo(referencia_resistencia_iat2);
  enviar_byte_config(';');

  // ========== TABELA VE ==========

  // d) Vetor MAP/TPS VE
  enviar_byte_config('d');
  enviar_byte_config(',');
  // vetor map ou tps da ve
  for (int i = 0; i < 16; i++) {
    enviar_campo(vetor_map_tps_ve[i]);

  }
  enviar_byte_config(';');

  // e) Vetor RPM VE
  enviar_byte_config('e');
  enviar_byte_config(',');
  // vetor rpm da tabela ve
  for (int i = 0; i < 16; i++) {
    enviar_campo(vetor_rpm_ve[i]);

  }
  enviar_byte_config(';');

  // f) Matriz VE (como vetor linear)
  enviar_byte_config('f');
  enviar_byte_config(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      enviar_campo(matriz_ve[i][j]);

    }
  }
  enviar_byte_config(';');

  // ========== CONFIGURAÇÕES INJEÇÃO E PROTEÇÃO ==========

  // m) Configuração injeção
  enviar_byte_config('m');
  enviar_byte_config(',');
  enviar_campo(referencia_leitura_injecao);
  enviar_campo(tipo_motor);
  enviar_campo(modo_injecao);
  enviar_campo(emparelhar_injetor);
  enviar_campo(deslocamento_motor);
  enviar_campo(numero_cilindro_injecao);
  enviar_campo(numero_injetor);
  enviar_campo(numero_esguicho);
  enviar_campo(tamanho_injetor);
  enviar_campo(tipo_acionamento_injetor);
  enviar_campo(tipo_combustivel);
  enviar_campo(REQ_FUEL);
  enviar_campo(dreq_fuel);
  enviar_campo(tipo_sonda_o2);
  enviar_byte_config(';');

  // t) Parametros do injetor
  enviar_byte_config('t');
  enviar_byte_config(',');
  enviar_campo(limite_injetor);
  enviar_campo(tempo_abertura_injetor);
  enviar_campo(grau_fechamento_injetor);
  enviar_campo(acrescimo_injecao_partida);
  enviar_campo(acrescimo_injecao_funcionamento);
  enviar_byte_config(';');

  // n) Configuração proteção e limites
  enviar_byte_config('n');
  enviar_byte_config(',');
  enviar_campo(tipo_protecao);
  enviar_campo(rpm_pre_corte);
  enviar_campo(avanco_corte);
  enviar_campo(tempo_corte);
  enviar_campo(rpm_maximo_corte);
  enviar_campo(numero_base_corte);
  enviar_campo(qtd_corte);
  enviar_byte_config(';');

    // o) Enriquecimento na aceleração
    enviar_byte_config('o');
    enviar_byte_config(',');
    enviar_campo(enriquecimento_aceleracao[0]);
    enviar_campo(enriquecimento_aceleracao[1]);
    enviar_campo(enriquecimento_aceleracao[2]);
    enviar_campo(enriquecimento_aceleracao[3]);
    enviar_campo(enriquecimento_aceleracao[4]);
    enviar_campo(tps_dot_escala[0]);
    enviar_campo(tps_dot_escala[1]);
    enviar_campo(tps_dot_escala[2]);
    enviar_campo(tps_dot_escala[3]);
    enviar_campo(tps_dot_escala[4]);
    enviar_campo(tipo_verificacao_aceleracao_rapida);
    enviar_campo(tps_mudanca_minima);
    enviar_campo(intervalo_tempo_aceleracao);
    enviar_campo(duracao_enriquecimento);
    enviar_campo(rpm_minimo_enriquecimento);
    enviar_campo(rpm_maximo_enriquecimento);
    enviar_campo(enriquecimento_desaceleracao);
    enviar_byte_config(';');

  // z) Tabela de offset por cilindro. Laco em vez de oito chamadas soltas:
  // custa bem menos flash, que na Nano e o recurso apertado.
  enviar_byte_config('z');
  enviar_byte_config(',');
  sendSerialInt(usar_offset_personalizado ? 1 : 0);
  for (byte i = 0; i < MAX_EVENTOS_AGENDAMENTO; i++) {
    enviar_byte_config(',');
    sendSerialInt(offset_evento[i]);
  }
  // Virgula final antes do ponto e virgula: e a convencao de todas as outras
  // secoes, e o parser da tela conta com ela - ele varre ate values.length-1
  // justamente para descartar o vazio que sobra. Sem a virgula, o ultimo
  // offset seria o descartado.
  enviar_byte_config(',');
  enviar_byte_config(';');

  // p) Configuração TPS
  enviar_byte_config('p');
  enviar_byte_config(',');
  enviar_campo(valor_tps_minimo);
  enviar_campo(valor_tps_maximo);
  enviar_byte_config(';');

  // q) Configuração MAP
    enviar_byte_config('q');
    enviar_byte_config(',');
    enviar_campo(valor_map_tipo);
    enviar_campo(valor_map_minimo);
    enviar_campo(valor_map_maximo);
    enviar_byte_config(';');

  // r) Enriquecimento de injeção por temperatura (5 pontos)
  enviar_byte_config('r');
  enviar_byte_config(',');
  for (int i = 0; i < 5; i++) {
    enviar_campo(vetor_temperatura_injecao[i]);
  }
  for (int i = 0; i < 5; i++) {
    enviar_campo(vetor_enriquecimento_temperatura[i]);
  }
  enviar_campo(usar_injecao_temperatura);
  enviar_byte_config(';');

  // s) Avanco por temperatura (5 pontos)
  enviar_byte_config('s');
  enviar_byte_config(',');
  for (int i = 0; i < 5; i++) {
    enviar_campo(vetor_temperatura[i]);
  }
  for (int i = 0; i < 5; i++) {
    enviar_campo(vetor_avanco_temperatura[i]);
  }
  enviar_campo(usar_avanco_temperatura);
  enviar_byte_config(';');

  // v) Configuracao de partida
  enviar_byte_config('v');
  enviar_byte_config(',');
  enviar_campo(rpm_partida);
  enviar_campo(nivel_limpeza_afogamento);
  enviar_campo(atraso_injecao_inicial);
  enviar_campo(tempo_reducao_enriquecimento_partida);
  enviar_byte_config(';');

  // w) Controle de marcha lenta
  enviar_byte_config('w');
  enviar_byte_config(',');
  enviar_campo(modo_marcha_lenta);
  enviar_campo(temperatura_desligamento_marcha_lenta);
  enviar_campo(histerese_marcha_lenta);
  enviar_campo(pwm_marcha_lenta_frio);
  enviar_campo(pwm_marcha_lenta_quente);
  enviar_campo(rpm_alvo_marcha_lenta);
  enviar_campo(maximo_passos_marcha_lenta);
  enviar_campo(inverter_direcao_marcha_lenta);
  enviar_byte_config(';');

  // V) Identidade: em que placa este firmware acha que esta.
  //
  // Vai por ultimo de proposito: quem le versoes antigas do firmware
  // simplesmente nao encontra a secao, em vez de encontrar o resto deslocado.
  //
  // perfil 1 = Autoino, 2 = Speeduino. A revisao e a da placa, e escolhe a
  // pinagem em definicoes_hardware.h - nao e rotulo.
  enviar_byte_config(';');
  enviar_byte_config('V');
  enviar_byte_config(',');
  enviar_campo(PERFIL_HARDWARE);
  enviar_campo(PLACA_REVISAO);
  enviar_byte_config(';');

#endif
}
