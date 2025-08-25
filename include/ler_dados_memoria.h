// Esta função envia os dados via serial no formato esperado
void sendSerialString(const char* str) {
  while (*str) {
    Serial.write(*str++);
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
  Serial.write(';');
  Serial.write('a');
  Serial.write(',');

  // Enviar os valores do vetor_map_tps
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_map_tps[i]);
    
      Serial.write(',');
    
  }
  Serial.write(';');
  // b) Vetor RPM ignição 
  Serial.write('b');
  Serial.write(',');
  // vetor rpm
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_rpm[i]);
    
      Serial.write(',');
    
  }
  Serial.write(';');

  // transforma matriz em vetor
  Serial.write('c');
  Serial.write(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      sendSerialInt(matriz_avanco[i][j]);
      
        Serial.write(',');
      
    }
  }
  Serial.write(';');
  // ========== CONFIGURAÇÕES SISTEMA ==========
    
  // g) Configuração inicial
  Serial.write('g');
  Serial.write(',');
  sendSerialInt(tipo_ignicao);
  Serial.write(',');
  sendSerialInt(qtd_dente);
  Serial.write(',');
  sendSerialInt(local_rodafonica);
  Serial.write(',');
  sendSerialInt(qtd_dente_faltante);
  Serial.write(',');
  sendSerialInt(grau_pms);
  Serial.write(',');
  sendSerialInt(qtd_cilindro * local_rodafonica);
  Serial.write(',');
  Serial.write(';');

  // j) Configuração faisca
  Serial.write('j');
  Serial.write(',');
  sendSerialInt(referencia_leitura_ignicao);
  Serial.write(',');
  sendSerialInt(modo_ignicao);
  Serial.write(',');
  sendSerialInt(grau_avanco_partida);
  Serial.write(',');
  sendSerialInt(avanco_fixo);
  Serial.write(',');
  sendSerialInt(grau_avanco_fixo);
  Serial.write(',');
  sendSerialInt(tipo_sinal_bobina);
  Serial.write(',');
  Serial.write(';');

  // k) Configuração dwell
  Serial.write('k');
  Serial.write(',');
  sendSerialInt(dwell_partida);
  Serial.write(',');
  sendSerialInt(dwell_funcionamento);
  Serial.write(',');
  Serial.write(';');

  // l) Configuração sensor temperatura CLT
  Serial.write('l');
  Serial.write(','); // letra L minúsculo
  sendSerialInt(referencia_temperatura_clt1);
  Serial.write(',');
  sendSerialInt(referencia_resistencia_clt1);
  Serial.write(',');
  sendSerialInt(referencia_temperatura_clt2);
  Serial.write(',');
  sendSerialInt(referencia_resistencia_clt2);
  Serial.write(',');
  Serial.write(';');

  // ========== TABELA VE ==========
    
  // d) Vetor MAP/TPS VE
  Serial.write('d');
  Serial.write(',');
  // vetor map ou tps da ve
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_map_tps_ve[i]);
    
      Serial.write(',');
    
  }
  Serial.write(';');

  // e) Vetor RPM VE
  Serial.write('e');
  Serial.write(',');
  // vetor rpm da tabela ve
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_rpm_ve[i]);
    
      Serial.write(',');
    
  }
  Serial.write(';');
  
  // f) Matriz VE (como vetor linear)
  Serial.write('f');
  Serial.write(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      sendSerialInt(matriz_ve[i][j]);
      
        Serial.write(',');
      
    }
  }
  Serial.write(';');

  // ========== CONFIGURAÇÕES INJEÇÃO E PROTEÇÃO ==========
    
  // m) Configuração injeção
  Serial.write('m');
  Serial.write(',');
  sendSerialInt(referencia_leitura_injecao);
  Serial.write(',');
  sendSerialInt(tipo_motor);
  Serial.write(',');
  sendSerialInt(modo_injecao);
  Serial.write(',');
  sendSerialInt(emparelhar_injetor);
  Serial.write(',');
  sendSerialInt(deslocamento_motor);
  Serial.write(',');
  sendSerialInt(numero_cilindro_injecao);
  Serial.write(',');
  sendSerialInt(numero_injetor);
  Serial.write(',');
  sendSerialInt(numero_esguicho);
  Serial.write(',');
  sendSerialInt(tamanho_injetor);
  Serial.write(',');
  sendSerialInt(tipo_acionamento_injetor);
  Serial.write(',');
  sendSerialInt(tipo_combustivel);
  Serial.write(',');
  sendSerialInt(REQ_FUEL);
  Serial.write(',');
  sendSerialInt(dreq_fuel);
  Serial.write(',');
  Serial.write(';');

  // n) Configuração proteção e limites 
  Serial.write('n');
  Serial.write(',');
  sendSerialInt(tipo_protecao);
  Serial.write(',');
  sendSerialInt(rpm_pre_corte);
  Serial.write(',');
  sendSerialInt(avanco_corte);
  Serial.write(',');
  sendSerialInt(tempo_corte);
  Serial.write(',');
  sendSerialInt(rpm_maximo_corte);
  Serial.write(',');
  sendSerialInt(numero_base_corte);
  Serial.write(',');
  sendSerialInt(qtd_corte);
  Serial.write(',');
  Serial.write(';');

    // o) Enriquecimento na aceleração
    Serial.write('o');
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[0]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[1]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[2]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[3]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[4]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[0]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[1]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[2]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[3]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[4]);
    Serial.write(',');
    sendSerialInt(tipo_verificacao_aceleracao_rapida);
    Serial.write(',');
    sendSerialInt(tps_mudanca_minima);
    Serial.write(',');
    sendSerialInt(intervalo_tempo_aceleracao);
    Serial.write(',');
    sendSerialInt(duracao_enriquecimento);
    Serial.write(',');
    sendSerialInt(rpm_minimo_enriquecimento);
    Serial.write(',');
    sendSerialInt(rpm_maximo_enriquecimento);
    Serial.write(',');
    sendSerialInt(enriquecimento_desaceleracao);
    Serial.write(',');
    Serial.write(';');

  // p) Configuração TPS
  Serial.write('p');
  Serial.write(',');
  sendSerialInt(valor_tps_minimo);
  Serial.write(',');
  sendSerialInt(valor_tps_maximo);
  Serial.write(',');
  Serial.write(';');

  // q) Configuração MAP
    Serial.write('q');
    Serial.write(',');
    sendSerialInt(valor_map_tipo);
    Serial.write(',');
    sendSerialInt(valor_map_minimo);
    Serial.write(',');
    sendSerialInt(valor_map_maximo);
    Serial.write(',');
    Serial.write(';');

}
