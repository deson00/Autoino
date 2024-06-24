void gravar_dados_eeprom_tabela_ignicao_map_rpm() {
  int highByte;
  int lowByte;
  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
  for (int i = 0; i < 16; i++) {
    highByte = vetor_rpm[i] >> 8; // Obtém o byte mais significativo
    lowByte = vetor_rpm[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.update(i * 2 + 10, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.update(i * 2 + 11, lowByte); // Armazena o byte menos significativo na posição i*2+11
  }
  // Escrita da matriz na EEPROM
  for (int i = 0; i < 16; i++) {
    EEPROM.update(i + 50, vetor_map_tps[i]); // Endereço de memória começa em 50
    for (int j = 0; j < 16; j++) {
      EEPROM.update(100 + i * 16 + j, matriz_avanco[i][j]); // Endereço de memória começa em 100, fim em 255 para a matriz
    }
  }
}

void gravar_dados_eeprom_configuracao_inicial() {
    int16_t grau_pms_16bit = (int16_t)grau_pms + 360; // Adiciona um offset de 360 para garantir que o valor seja sempre positivo
    if (grau_pms_16bit < 0) { // Verifica se o valor é negativo
        grau_pms_16bit = ~grau_pms_16bit + 1; // Calcula o valor em complemento de dois
    }
    uint8_t highByte = (uint8_t)(grau_pms_16bit >> 8); // Obtém o byte mais significativo
    uint8_t lowByte = (uint8_t)(grau_pms_16bit & 0xFF); // Obtém o byte menos significativo
    EEPROM.update(0*2+360, tipo_ignicao); // Armazena o byte na posição i*2+260
    EEPROM.update(1*2+360, qtd_dente);
    EEPROM.update(2*2+360, local_rodafonica);
    EEPROM.update(3*2+360, qtd_dente_faltante);
    EEPROM.update(4*2+360, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.update(4*2+361, lowByte); // Armazena o byte menos significativo na posição i*2+11
    EEPROM.update(5*2+360, qtd_cilindro);
}

void gravar_dados_eeprom_configuracao_faisca() {
    EEPROM.update(1*2+380, referencia_leitura_ignicao);
    EEPROM.update(2*2+380, modo_ignicao);
    EEPROM.update(3*2+380, grau_avanco_partida);
    EEPROM.update(4*2+380, avanco_fixo);
    EEPROM.update(5*2+380, grau_avanco_fixo);
    EEPROM.update(6*2+380, tipo_sinal_bobina); 
}

void gravar_dados_eeprom_configuracao_dwell() {
    EEPROM.update(1*2+400, dwell_partida);
    EEPROM.update(2*2+400, dwell_funcionamento); 
}

void gravar_dados_eeprom_configuracao_clt() {
    // Gravar os valores divididos em bytes
    EEPROM.update(410, referencia_temperatura_clt1 & 0xFF);        // Byte menos significativo
    //EEPROM.update(411, (referencia_temperatura_clt1 >> 8) & 0xFF);  // Byte mais significativo
    EEPROM.update(412, referencia_resistencia_clt1 & 0xFF);
    EEPROM.update(413, (referencia_resistencia_clt1 >> 8) & 0xFF);
    EEPROM.update(414, referencia_temperatura_clt2 & 0xFF);
    //EEPROM.update(415, (referencia_temperatura_clt2 >> 8) & 0xFF);
    EEPROM.update(416, referencia_resistencia_clt2 & 0xFF);
    EEPROM.update(417, (referencia_resistencia_clt2 >> 8) & 0xFF);
}

void gravar_dados_eeprom_tabela_ve_map_rpm() {
  int highByte;
  int lowByte;
  int endereco = 418; // Inicializa o endereço de memória

  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
  for (int i = 0; i < 16; i++) {
    highByte = vetor_rpm_ve[i] >> 8; // Obtém o byte mais significativo
    lowByte = vetor_rpm_ve[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.update(endereco, highByte); // Armazena o byte mais significativo no endereço atual
    EEPROM.update(endereco + 1, lowByte); // Armazena o byte menos significativo no próximo endereço
    endereco += 2; // Avança para o próximo par de endereços
  }

  // Escrita da matriz na EEPROM
  endereco = 450; // Define o novo endereço de memória para a matriz

  for (int i = 0; i < 16; i++) {
    EEPROM.update(endereco, vetor_map_tps_ve[i]); // Armazena o valor do vetor na posição atual
    endereco++; // Avança para o próximo endereço

    for (int j = 0; j < 16; j++) {
      EEPROM.update(endereco, matriz_ve[i][j]); // Armazena o valor da matriz na posição atual
      endereco++; // Avança para o próximo endereço
    }
  }
}

void gravar_dados_eeprom_configuracao_injecao(){
  int endereco =  722; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    EEPROM.update(endereco++, referencia_leitura_injecao); 
    EEPROM.update(endereco++, tipo_motor);
    EEPROM.update(endereco++, modo_injecao);
    EEPROM.update(endereco++, emparelhar_injetor);
    EEPROM.update(endereco++, deslocamento_motor & 0xFF);
    EEPROM.update(endereco++, (deslocamento_motor >> 8) & 0xFF);
    EEPROM.update(endereco++, numero_cilindro_injecao);
    EEPROM.update(endereco++, numero_injetor);
    EEPROM.update(endereco++, numero_esguicho);
    EEPROM.update(endereco++, tamanho_injetor & 0xFF);
    EEPROM.update(endereco++, (tamanho_injetor >> 8) & 0xFF);
    EEPROM.update(endereco++, tipo_acionamento_injetor);
    EEPROM.update(endereco++, tipo_combustivel & 0xFF);
    EEPROM.update(endereco++, (tipo_combustivel >> 8) & 0xFF);
    EEPROM.update(endereco++, REQ_FUEL & 0xFF);
    EEPROM.update(endereco++, (REQ_FUEL >> 8) & 0xFF);
    EEPROM.update(endereco++, (dreq_fuel & 0xFF));
    EEPROM.update(endereco++, (dreq_fuel >> 8) & 0xFF);
    
}

void gravar_dados_eeprom_configuracao_protecao(){
  int endereco =  750; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    EEPROM.update(endereco++, tipo_protecao); 
    EEPROM.update(endereco++, rpm_pre_corte & 0xFF);
    EEPROM.update(endereco++, (rpm_pre_corte >> 8) & 0xFF);
    EEPROM.update(endereco++, avanco_corte);
    EEPROM.update(endereco++, tempo_corte);
    EEPROM.update(endereco++, rpm_maximo_corte & 0xFF);
    EEPROM.update(endereco++, (rpm_maximo_corte >> 8) & 0xFF);
    EEPROM.update(endereco++, numero_base_corte);
    EEPROM.update(endereco++, qtd_corte);
}
