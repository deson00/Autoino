//função de leitura dos dados na porta serial
void leitura_entrada_dados_serial()
{
  if (Serial.available() > 0){
    char data = Serial.read();
    if (data == 'a'){//entrada de dados do vetor map ou tps
      tipo_vetor_map_tps_avanco = 1;
      indice_vetor_entrada_dados_serial = 0;
    }
    if (data == 'b'){//entrada de dados do vetor rpm
      tipo_vetor_rpm_avanco = 1;//b,500,600,700,800,900,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000,11000; exmplo
    }
    if (data == 'c'){//entrada de dados do da matriz em vetor
      tipo_vetor_matriz_avanco = 1;
    }
    if (data == 'd'){// entrada vetor map ou tps para a tabela ve
      tipo_vetor_map_tps_ve = 1;
    }
    if (data == 'e'){//entrada vetor rpm para a tabela ve
      tipo_vetor_rpm_ve = 1;
    }
    if (data == 'f'){//entrada da matriz de valores para a tabela ve
      tipo_vetor_matriz_ve = 1;  
    }
    if (data == 'g'){//grava configuração inicial
      tipo_vetor_configuracao_inicial = 1;
    }
     if (data == 'h'){//retorna dados da ecu
        //ler_dados_eeprom();
        ler_dados_memoria();
    }
    if (data == 'i') {
     if(status_dados_tempo_real){
      status_dados_tempo_real = false;
     }else{
      status_dados_tempo_real = true;
     }
    }
    if (data == 'j') {// configuração da faisca
      tipo_vetor_configuracao_faisca = 1;
    }
    if (data == 'k') {// configuração da faisca
      tipo_vetor_configuracao_dwell = 1;
    }
    if (data == 'l') {// configuração sensor temperatura clt
      tipo_vetor_configuracao_clt = 1;
    }
    if (data == 'm') {// configuração sensor temperatura clt
      tipo_vetor_configuracao_injecao = 1;
    }
    if (data == 'n') {// configuração proteção
      tipo_vetor_protecao = 1;
    }
    if (data == 'o') {// configuração proteção
      tipo_vetor_enriquecimento_aceleracao = 1;
    }
    if (data == 'p') {// configuração TPS
      tipo_vetor_configuracao_tps = 1;
    }
    if (data == 'q') {// configuração MAP
      tipo_vetor_configuracao_map = 1;
    }
    if (data == 'z') {// configuração da ve e ponto
       gravar_dados_eeprom_tabela_ignicao_map_rpm();
       gravar_dados_eeprom_tabela_ve_map_rpm();
    }
    if (data == ';'){ // final do vetor
      if (tipo_vetor_map_tps_avanco){
        // for (int i = 0; i < 16; i++){
        //   vetor_map_tps[i] = values[i];
        // }
        indice_vetor_entrada_dados_serial = 0;
        tipo_vetor_map_tps_avanco = 0;
      }
      if (tipo_vetor_rpm_avanco){
        // for (int i = 0; i < 16; i++){
        //   vetor_rpm[i] = values[i];
        // }
        indice_vetor_entrada_dados_serial = 0;
        tipo_vetor_rpm_avanco = 0;
      }
      if (tipo_vetor_matriz_avanco){
        // // transforma o vetor em matriz
        // int k = 0;
        // for (int i = 0; i < 16; i++){
        //   for (int j = 0; j < 16; j++){
        //     matriz_avanco[i][j] = values[k];
        //     k++;
        //   }
        // }
        indice_vetor_entrada_dados_serial = 0;
        indice_matrix_entrada_dados_seriala = 0; // Índice usado para rastrear a linha atual na matriz
        indice_matrix_entrada_dados_serialb = 0; // Índice usado para rastrear a coluna atual na matriz
        tipo_vetor_matriz_avanco = 0;
      }
      if (tipo_vetor_map_tps_ve){
        // for (int i = 0; i < 16; i++){
        //   vetor_map_tps_ve[i] = values[i];
        // }
        indice_vetor_entrada_dados_serial = 0;
        tipo_vetor_map_tps_ve = 0;
      }
      if (tipo_vetor_rpm_ve){
        // for (int i = 0; i < 16; i++){
        //   vetor_rpm_ve[i] = values[i];
        // }
        indice_vetor_entrada_dados_serial = 0;
        tipo_vetor_rpm_ve = 0;
      }
      if (tipo_vetor_matriz_ve){
        // // transforma o vetor em matriz
        // int k = 0;
        // for (int i = 0; i < 16; i++){
        //   for (int j = 0; j < 16; j++){
        //     matriz_ve[i][j] = values[k];
        //     k++;
        //   }
        // }
        indice_vetor_entrada_dados_serial = 0;
        indice_matrix_entrada_dados_seriala = 0; // Índice usado para rastrear a linha atual na matriz
        indice_matrix_entrada_dados_serialb = 0; // Índice usado para rastrear a coluna atual na matriz
        tipo_vetor_matriz_ve = 0;
      }
      if (tipo_vetor_configuracao_inicial == 1){
          tipo_ignicao = values[0];
          qtd_dente = values[1];
          local_rodafonica = values[2]; // 2 para virabrequinho e 1 para comando
          qtd_dente_faltante = values[3];
          grau_pms = values[4];
          qtd_cilindro = values[5] / local_rodafonica;
          grau_entre_cada_cilindro = 360 / qtd_cilindro;
          gravar_dados_eeprom_configuracao_inicial();
          tipo_vetor_configuracao_inicial = 0;
      }
      if (tipo_vetor_configuracao_faisca == 1){
          referencia_leitura_ignicao = values[0];//map 1 e tps 2
          modo_ignicao = values[1]; // 1 para centelha perdida e 2 para centelha unica
          grau_avanco_partida = values[2]; // avanço definido apenas na partida 
          avanco_fixo = values[3]; // avanço fixo 0 desligado e 1 ligado
          grau_avanco_fixo = values[4]; // grau de avanço fixo de 0 a 360 mais usado para calibrar o pms
          tipo_sinal_bobina = values[5] ; // 1 alto e 0 baixo tipo de sinal enviado para bobina ente alto ou baixo conforme modelo da bobina
          gravar_dados_eeprom_configuracao_faisca();
          tipo_vetor_configuracao_faisca = 0;
      }
      if (tipo_vetor_configuracao_dwell == 1){
          dwell_partida = values[0];
          dwell_funcionamento = values[1];
          gravar_dados_eeprom_configuracao_dwell();
          tipo_vetor_configuracao_dwell = 0;
      }
      if (tipo_vetor_configuracao_clt == 1){
          referencia_temperatura_clt1 = values[0];
          referencia_resistencia_clt1 = values[1];
          referencia_temperatura_clt2 = values[2];
          referencia_resistencia_clt2 = values[3];
          gravar_dados_eeprom_configuracao_clt();
          tipo_vetor_configuracao_clt = 0;
      }
      if (tipo_vetor_configuracao_injecao == 1){
          referencia_leitura_injecao = values[0];//1 map 2 tps
          tipo_motor = values[1];//4 para motor 4 tempo e 2 para 2 tempo
          modo_injecao = values[2]; // 1 pareado 2 semi e 3 sequencial
          emparelhar_injetor = values[3];// 1 para 1234 para emparelhado 2 para 1423 semi ou sequencial e 3 para 1342 para semi ou sequencial
          deslocamento_motor = values[4];// tamanho do motor em polegadas 
          numero_cilindro_injecao = values[5];
          numero_injetor = values[6];
          numero_esguicho = values[7];
          tamanho_injetor = values[8];
          tipo_acionamento_injetor = values[9];
          tipo_combustivel = values[10];
          REQ_FUEL = values[11];
          dreq_fuel = values[12];
          gravar_dados_eeprom_configuracao_injecao();
          tipo_vetor_configuracao_injecao = 0;
      }
      if (tipo_vetor_protecao == 1){
          tipo_protecao = values[0];//0 desligado, 1 apenas igniçao, 2 apenas injeção e 3 ignição e injeção
          rpm_pre_corte = values[1];
          avanco_corte = values[2]; 
          tempo_corte = values[3]; 
          rpm_maximo_corte = values[4];
          numero_base_corte = values[5];
          qtd_corte = values[6];
          gravar_dados_eeprom_configuracao_protecao();
          tipo_vetor_protecao = 0;
      }
      if (tipo_vetor_enriquecimento_aceleracao == 1){
          enriquecimento_aceleracao[0] = values[0];
          enriquecimento_aceleracao[1] = values[1];
          enriquecimento_aceleracao[2] = values[2]; 
          enriquecimento_aceleracao[3] = values[3]; 
          enriquecimento_aceleracao[4] = values[4];
          tps_dot_escala[0] = values[5];
          tps_dot_escala[1] = values[6];
          tps_dot_escala[2] = values[7];
          tps_dot_escala[3] = values[8];
          tps_dot_escala[4] = values[9];
          tipo_verificacao_aceleracao_rapida = values[10];
          tps_mudanca_minima = values[11];
          intervalo_tempo_aceleracao = values[12];
          duracao_enriquecimento = values[13];
          rpm_minimo_enriquecimento = values[14];
          rpm_maximo_enriquecimento = values[15];
          enriquecimento_desaceleracao = values[16];
          gravar_dados_eeprom_enriquecimento_aceleracao();
          tipo_vetor_enriquecimento_aceleracao = 0;
      }
      if (tipo_vetor_configuracao_tps == 1){
          valor_tps_minimo = values[0];
          valor_tps_maximo = values[1];
          gravar_dados_eeprom_configuracao_tps();
          tipo_vetor_configuracao_tps = 0;
      }
      if (tipo_vetor_configuracao_map == 1){
          valor_map_tipo = values[0];
          valor_map_minimo = values[1];
          valor_map_maximo = values[2];
          gravar_dados_eeprom_configuracao_map();
          tipo_vetor_configuracao_map = 0;
      }
      index = 0; // reinicia índice do vetor
    }
    else if (isdigit(data)){                                
      buffer[strlen(buffer)] = data; // adiciona o caractere recebido no buffer temporário
      if (strlen(buffer) >= sizeof(buffer) - 1){// verifica se o buffer está cheio
        buffer[sizeof(buffer) - 1] = '\0'; // adiciona um terminador de string para evitar um buffer overflow
      }
    }
    else if ((data == ',' || data == ';') && strlen(buffer) > 0){                                 
      buffer[strlen(buffer)] = '\0';  // adiciona um terminador de string para converter o buffer em uma string válida
      //values[index++] = atoi(buffer); // adiciona ao vetor
      //Substituído por strtol
      values[index++] = strtol(buffer, NULL, 10);
      //--------tabela ignção ----------//
      if (tipo_vetor_map_tps_avanco){
          vetor_map_tps[indice_vetor_entrada_dados_serial] = strtol(buffer, NULL, 10);
          indice_vetor_entrada_dados_serial++;
      }
      if (tipo_vetor_rpm_avanco){
        vetor_rpm[indice_vetor_entrada_dados_serial] = strtol(buffer, NULL, 10);
        indice_vetor_entrada_dados_serial++;
      }
      if (tipo_vetor_matriz_avanco){
        if (indice_matrix_entrada_dados_seriala < 16) {
            matriz_avanco[indice_matrix_entrada_dados_seriala][indice_matrix_entrada_dados_serialb] = strtol(buffer, NULL, 10);
          if (indice_matrix_entrada_dados_serialb < 15) {
            indice_matrix_entrada_dados_serialb++; // Avança para a próxima coluna
          } else {
            indice_matrix_entrada_dados_seriala++; // Se chegou ao fim da linha atual, avança para a próxima linha
            indice_matrix_entrada_dados_serialb = 0; // Reinicia o contador de colunas
          }
        } 
      }
      //---------tabela ve --------//
      if (tipo_vetor_map_tps_ve){
          vetor_map_tps_ve[indice_vetor_entrada_dados_serial] = strtol(buffer, NULL, 10);
          indice_vetor_entrada_dados_serial++;
      }
      if (tipo_vetor_rpm_ve){
        vetor_rpm_ve[indice_vetor_entrada_dados_serial] = strtol(buffer, NULL, 10);
        indice_vetor_entrada_dados_serial++;
      }
      if (tipo_vetor_matriz_ve){
        if (indice_matrix_entrada_dados_seriala < 16) {
            matriz_ve[indice_matrix_entrada_dados_seriala][indice_matrix_entrada_dados_serialb] = strtol(buffer, NULL, 10);
          if (indice_matrix_entrada_dados_serialb < 15) {
            indice_matrix_entrada_dados_serialb++; // Avança para a próxima coluna
          } else {
            indice_matrix_entrada_dados_seriala++; // Se chegou ao fim da linha atual, avança para a próxima linha
            indice_matrix_entrada_dados_serialb = 0; // Reinicia o contador de colunas
          }
        } 
      }
      if (index >= maximo_valores_recebido){
        index = 0; // reinicia índice do vetor se estiver cheio
      }
      memset(buffer, 0, sizeof(buffer)); // reinicia o buffer
    }
  }
}