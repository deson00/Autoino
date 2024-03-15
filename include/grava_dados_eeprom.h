void gravar_dados_eeprom_tabela_ignicao_map_rpm() {
  int highByte;
  int lowByte;
  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
  for (int i = 0; i < 16; i++) {
    highByte = vetor_rpm[i] >> 8; // Obtém o byte mais significativo
    lowByte = vetor_rpm[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.write(i * 2 + 10, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.write(i * 2 + 11, lowByte); // Armazena o byte menos significativo na posição i*2+11
  }
  // Escrita da matriz na EEPROM
  for (int i = 0; i < 16; i++) {
    EEPROM.write(i + 50, vetor_map_tps[i]); // Endereço de memória começa em 50
    for (int j = 0; j < 16; j++) {
      EEPROM.write(100 + i * 16 + j, matriz_avanco[i][j]); // Endereço de memória começa em 100, fim em 255 para a matriz
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
    EEPROM.write(0*2+360, tipo_ignicao); // Armazena o byte na posição i*2+260
    EEPROM.write(1*2+360, qtd_dente);
    EEPROM.write(2*2+360, local_rodafonica);
    EEPROM.write(3*2+360, qtd_dente_faltante);
    EEPROM.write(4*2+360, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.write(4*2+361, lowByte); // Armazena o byte menos significativo na posição i*2+11
    EEPROM.write(5*2+360, qtd_cilindro);
}

void gravar_dados_eeprom_configuracao_faisca() {
    EEPROM.write(1*2+380, referencia_leitura_ignicao);
    EEPROM.write(2*2+380, modo_ignicao);
    EEPROM.write(3*2+380, grau_avanco_partida);
    EEPROM.write(4*2+380, avanco_fixo);
    EEPROM.write(5*2+380, grau_avanco_fixo);
    EEPROM.write(6*2+380, tipo_sinal_bobina); 
}

void gravar_dados_eeprom_configuracao_dwell() {
    EEPROM.write(1*2+400, dwell_partida);
    EEPROM.write(2*2+400, dwell_funcionamento); 
}

void gravar_dados_eeprom_configuracao_clt() {
    // Gravar os valores divididos em bytes
    EEPROM.write(410, referencia_temperatura_clt1 & 0xFF);        // Byte menos significativo
    //EEPROM.write(411, (referencia_temperatura_clt1 >> 8) & 0xFF);  // Byte mais significativo
    EEPROM.write(412, referencia_resistencia_clt1 & 0xFF);
    EEPROM.write(413, (referencia_resistencia_clt1 >> 8) & 0xFF);
    EEPROM.write(414, referencia_temperatura_clt2 & 0xFF);
    //EEPROM.write(415, (referencia_temperatura_clt2 >> 8) & 0xFF);
    EEPROM.write(416, referencia_resistencia_clt2 & 0xFF);
    EEPROM.write(417, (referencia_resistencia_clt2 >> 8) & 0xFF);
}

void gravar_dados_eeprom_tabela_ve_map_rpm() {
  int highByte;
  int lowByte;
  int endereco = 418; // Inicializa o endereço de memória

  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
  for (int i = 0; i < 16; i++) {
    highByte = vetor_rpm_ve[i] >> 8; // Obtém o byte mais significativo
    lowByte = vetor_rpm_ve[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.write(endereco, highByte); // Armazena o byte mais significativo no endereço atual
    EEPROM.write(endereco + 1, lowByte); // Armazena o byte menos significativo no próximo endereço
    endereco += 2; // Avança para o próximo par de endereços
  }

  // Escrita da matriz na EEPROM
  endereco = 450; // Define o novo endereço de memória para a matriz

  for (int i = 0; i < 16; i++) {
    EEPROM.write(endereco, vetor_map_tps_ve[i]); // Armazena o valor do vetor na posição atual
    endereco++; // Avança para o próximo endereço

    for (int j = 0; j < 16; j++) {
      EEPROM.write(endereco, matriz_ve[i][j]); // Armazena o valor da matriz na posição atual
      endereco++; // Avança para o próximo endereço
    }
  }
}

void gravar_dados_eeprom_configuracao_injecao(){
  int endereco =  722; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    EEPROM.write(endereco++, referencia_leitura_injecao); 
    EEPROM.write(endereco++, tipo_motor);
    EEPROM.write(endereco++, modo_injecao);
    EEPROM.write(endereco++, emparelhar_injetor);
    EEPROM.write(endereco++, deslocamento_motor & 0xFF);
    EEPROM.write(endereco++, (deslocamento_motor >> 8) & 0xFF);
    EEPROM.write(endereco++, numero_cilindro_injecao);
    EEPROM.write(endereco++, numero_injetor);
    EEPROM.write(endereco++, numero_esguicho);
    EEPROM.write(endereco++, tamanho_injetor & 0xFF);
    EEPROM.write(endereco++, (tamanho_injetor >> 8) & 0xFF);
    EEPROM.write(endereco++, tipo_acionamento_injetor);
    EEPROM.write(endereco++, tipo_combustivel & 0xFF);
    EEPROM.write(endereco++, (tipo_combustivel >> 8) & 0xFF);
    EEPROM.write(endereco++, REQ_FUEL & 0xFF);
    EEPROM.write(endereco++, (REQ_FUEL >> 8) & 0xFF);
    EEPROM.write(endereco++, (dreq_fuel & 0xFF));
    EEPROM.write(endereco++, (dreq_fuel >> 8) & 0xFF);
    
}
