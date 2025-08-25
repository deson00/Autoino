
void ler_dados_eeprom_tabela_ignicao_map_rpm() {
    int endereco = 10; // Endereço base

    // 1. Ler vetor_rpm (16 valores de 16 bits cada)
    for (int i = 0; i < 16; i++) {
        vetor_rpm[i] = ler_16bits_eeprom(endereco);
        endereco += 2;
    }

    // 2. Ler vetor_map_tps (16 valores de 8 bits cada)
    endereco = 50; // Força endereço como no código original
    for (int i = 0; i < 16; i++) {
        vetor_map_tps[i] = ler_8bits_eeprom(endereco);
        endereco++;
    }

    // 3. Ler matriz_avanco (16x16 valores de 8 bits cada = 256 bytes)
    endereco = 100; // Força endereço como no código original
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            matriz_avanco[i][j] = ler_8bits_eeprom(endereco);
            endereco++;
        }
    }
}
void ler_dados_eeprom_tabela_ve_map_rpm() {
    int endereco = 500; // Endereço base

    // 1. Ler vetor_rpm_ve (16 valores de 16 bits cada)
    for (int i = 0; i < 16; i++) {
        vetor_rpm_ve[i] = ler_16bits_eeprom(endereco);
        endereco += 2;
    }
    // endereco agora = 532

    // 2. Ler vetor_map_tps_ve (16 valores de 8 bits cada)
    for (int i = 0; i < 16; i++) {
        vetor_map_tps_ve[i] = ler_8bits_eeprom(endereco);
        endereco++;
    }
    // endereco agora = 548

    // 3. Ler matriz_ve (16x16 valores de 8 bits cada = 256 bytes)
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            matriz_ve[i][j] = ler_8bits_eeprom(endereco);
            endereco++;
        }
    }
    // endereco final = 804
}

void ler_dados_eeprom_configuracao_injecao(){
  int endereco = 900; // Inicializa o endereço de memória

  // Ler os valores divididos em bytes
  referencia_leitura_injecao = EEPROM.read(endereco++);
  tipo_motor = EEPROM.read(endereco++);
  modo_injecao = EEPROM.read(endereco++);
  emparelhar_injetor = EEPROM.read(endereco++);
  
  // Para deslocamento_motor, combinamos os bytes lidos
  // deslocamento_motor (16 bits)
  deslocamento_motor = ler_16bits_eeprom(endereco);
  endereco += 2;
  
  numero_cilindro_injecao = EEPROM.read(endereco++);
  numero_injetor = EEPROM.read(endereco++);
  numero_esguicho = EEPROM.read(endereco++);
  
  // Para tamanho_injetor, combinamos os bytes lidos
  // tamanho_injetor (16 bits)
  tamanho_injetor = ler_16bits_eeprom(endereco);
  endereco += 2;
  
  tipo_acionamento_injetor = EEPROM.read(endereco++);
  // tipo_combustivel (16 bits)
  tipo_combustivel = ler_16bits_eeprom(endereco);
  endereco += 2;
  
  // REQ_FUEL (16 bits)
  REQ_FUEL = ler_16bits_eeprom(endereco);
  endereco += 2;
  
  // dreq_fuel (16 bits)
  dreq_fuel = ler_16bits_eeprom(endereco);
  endereco += 2;
}


void ler_dados_eeprom_configuracao_protecao(){
  int endereco =  950; // Inicializa o endereço de memória
  // Ler os valores e atribuir às variáveis correspondentes
  tipo_protecao = EEPROM.read(endereco++);
  // rpm_pre_corte (16 bits)
  rpm_pre_corte = ler_16bits_eeprom(endereco);
  endereco += 2;
  avanco_corte = EEPROM.read(endereco++);
  tempo_corte = EEPROM.read(endereco++);
  // rpm_maximo_corte (16 bits)
  rpm_maximo_corte = ler_16bits_eeprom(endereco);
  endereco += 2;
  numero_base_corte = EEPROM.read(endereco++);
  qtd_corte = EEPROM.read(endereco++);
}

void ler_dados_eeprom_enriquecimento_aceleracao() {
    int endereco = 970; // Inicializa o endereço de memória

    // Ler os valores de enriquecimento_aceleracao (1 byte cada)
    for (int i = 0; i < 5; i++) {
        enriquecimento_aceleracao[i] = EEPROM.read(endereco++);
    }

    // Ler os valores de tps_dot_escala (2 bytes cada)
    for (int i = 0; i < 5; i++) {
      tps_dot_escala[i] = ler_16bits_eeprom(endereco);
      endereco += 2;
    }

    // Ler os valores dos parâmetros restantes
    tipo_verificacao_aceleracao_rapida = EEPROM.read(endereco++);
    tps_mudanca_minima = EEPROM.read(endereco++);
        // Valores de 16 bits
    intervalo_tempo_aceleracao = ler_16bits_eeprom(endereco);
    endereco += 2;
    duracao_enriquecimento = ler_16bits_eeprom(endereco);
    endereco += 2;
    rpm_minimo_enriquecimento = ler_16bits_eeprom(endereco);
    endereco += 2;
    rpm_maximo_enriquecimento = ler_16bits_eeprom(endereco);
    endereco += 2;    
    enriquecimento_desaceleracao = EEPROM.read(endereco++);

}
void leitura_dados_eeprom_configuracao_tps() {
    int endereco = 1000; // Inicializa o endereço de memória
    // Ler os valores dos parâmetros
    valor_tps_minimo = ler_16bits_eeprom(endereco);
    endereco += 2;
    
    valor_tps_maximo = ler_16bits_eeprom(endereco);
    endereco += 2;
}
void ler_dados_eeprom_configuracao_map() {
    int endereco = 1010; // ENDEREÇO CORRIGIDO
    valor_map_tipo = EEPROM.read(endereco++);
    valor_map_minimo = ler_16bits_eeprom(endereco);
    endereco += 2;
    valor_map_maximo = ler_16bits_eeprom(endereco);
    endereco += 2;
}

void ler_dados_eeprom_configuracao_inicial() {
    tipo_ignicao = ler_8bits_eeprom(360);
    qtd_dente = ler_8bits_eeprom(362);
    local_rodafonica = ler_8bits_eeprom(364);
    qtd_dente_faltante = ler_8bits_eeprom(366);
    grau_pms = ler_16bits_eeprom(368) - 360; // Simplesmente remove offset
    qtd_cilindro = ler_8bits_eeprom(370);
    
    grau_entre_cada_cilindro = 360 / qtd_cilindro;
    grau_cada_dente = 360 / qtd_dente;
}
void ler_dados_eeprom(){
    ler_dados_eeprom_tabela_ignicao_map_rpm();
    ler_dados_eeprom_tabela_ve_map_rpm();
    ler_dados_eeprom_configuracao_injecao();
    ler_dados_eeprom_configuracao_protecao();
    ler_dados_eeprom_enriquecimento_aceleracao();
    leitura_dados_eeprom_configuracao_tps();
    ler_dados_eeprom_configuracao_map();
    ler_dados_eeprom_configuracao_inicial();    
    
    // Leitura dos dados de configurações de faisca 
    int endereco = 382;
    referencia_leitura_ignicao = ler_8bits_eeprom(endereco); endereco += 2;
    modo_ignicao = ler_8bits_eeprom(endereco); endereco += 2;
    grau_avanco_partida = ler_8bits_eeprom(endereco); endereco += 2;
    avanco_fixo = ler_8bits_eeprom(endereco); endereco += 2;
    grau_avanco_fixo = ler_8bits_eeprom(endereco); endereco += 2;
    tipo_sinal_bobina = ler_8bits_eeprom(endereco);

    // Leitura dos dados de configurações de dwell 
    endereco = 402;
    dwell_partida = ler_8bits_eeprom(endereco); endereco += 2;
    dwell_funcionamento = ler_8bits_eeprom(endereco);

    // Leitura dos dados de calibração de CLT
    referencia_temperatura_clt1 = ler_8bits_eeprom(410);
    referencia_resistencia_clt1 = ler_16bits_eeprom(412);
    referencia_temperatura_clt2 = ler_8bits_eeprom(414);
    referencia_resistencia_clt2 = ler_16bits_eeprom(416); 

}

