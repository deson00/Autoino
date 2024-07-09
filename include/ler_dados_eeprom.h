
int ler_valor_eeprom_2byte(int endereco) {
    // Lê os dois bytes da EEPROM
    byte byte_menos_significativo = EEPROM.read(endereco);
    byte byte_mais_significativo = EEPROM.read(endereco + 1);
    // Reconstrói o valor inteiro a partir dos bytes lidos
    int valor = byte_mais_significativo << 8 | byte_menos_significativo;
    return valor;
}

void ler_dados_eeprom_tabela_ve_map_rpm() {
  int highByte;
  int lowByte;
  int endereco =  418; // Inicializa o endereço de memória para vetor_rpm_ve

  // Leitura do vetor_rpm de valores de  2 bytes na EEPROM
  for (int i =  0; i <  16; i++) {
    highByte = EEPROM.read(endereco); // Lê o byte mais significativo
    lowByte = EEPROM.read(endereco +  1); // Lê o byte menos significativo
    vetor_rpm_ve[i] = (highByte <<  8) | lowByte; // Combina os bytes para formar o valor inteiro
    endereco +=  2; // Avança para o próximo par de endereços
  }

  // Leitura da matriz na EEPROM
  endereco =  450; // Define o novo endereço de memória para a matriz

  for (int i =  0; i <  16; i++) {
    vetor_map_tps_ve[i] = EEPROM.read(endereco); // Lê o valor do vetor na posição atual
    endereco++; // Avança para o próximo endereço

    for (int j =  0; j <  16; j++) {
      matriz_ve[i][j] = EEPROM.read(endereco); // Lê o valor da matriz na posição atual
      endereco++; // Avança para o próximo endereço
    }
  }
}




void ler_dados_eeprom_configuracao_injecao(){
  int endereco = 722; // Inicializa o endereço de memória

  // Ler os valores divididos em bytes
  referencia_leitura_injecao = EEPROM.read(endereco++);
  tipo_motor = EEPROM.read(endereco++);
  modo_injecao = EEPROM.read(endereco++);
  emparelhar_injetor = EEPROM.read(endereco++);
  
  // Para deslocamento_motor, combinamos os bytes lidos
  byte lowByte = EEPROM.read(endereco++);
  byte highByte = EEPROM.read(endereco++);
  deslocamento_motor = lowByte | (highByte << 8);
  
  numero_cilindro_injecao = EEPROM.read(endereco++);
  numero_injetor = EEPROM.read(endereco++);
  numero_esguicho = EEPROM.read(endereco++);
  
  // Para tamanho_injetor, combinamos os bytes lidos
  lowByte = EEPROM.read(endereco++);
  highByte = EEPROM.read(endereco++);
  tamanho_injetor = lowByte | (highByte << 8);
  
  tipo_acionamento_injetor = EEPROM.read(endereco++);
  // Para tipo combustivel, combinamos os bytes lidos
  lowByte = EEPROM.read(endereco++);
  highByte = EEPROM.read(endereco++);
  tipo_combustivel = lowByte | (highByte << 8);
  
  // Para REQ_FUEL, combinamos os bytes lidos
  lowByte = EEPROM.read(endereco++);
  highByte = EEPROM.read(endereco++);
  REQ_FUEL = lowByte | (highByte << 8);
  
  // Para dreq_fuel, combinamos os bytes lidos
  lowByte = EEPROM.read(endereco++);
  highByte = EEPROM.read(endereco++);
  dreq_fuel = lowByte | (highByte << 8);
}


void ler_dados_eeprom_configuracao_protecao(){
  int endereco =  750; // Inicializa o endereço de memória
  // Ler os valores e atribuir às variáveis correspondentes
  tipo_protecao = EEPROM.read(endereco++);
  rpm_pre_corte = EEPROM.read(endereco++);
  rpm_pre_corte |= EEPROM.read(endereco++) << 8;
  avanco_corte = EEPROM.read(endereco++);
  tempo_corte = EEPROM.read(endereco++);
  rpm_maximo_corte = EEPROM.read(endereco++);
  rpm_maximo_corte |= EEPROM.read(endereco++) << 8;
  numero_base_corte = EEPROM.read(endereco++);
  qtd_corte = EEPROM.read(endereco++);
}

void ler_dados_eeprom_enriquecimento_aceleracao() {
    int endereco = 770; // Inicializa o endereço de memória

    // Ler os valores de enriquecimento_aceleracao (1 byte cada)
    for (int i = 0; i < 5; i++) {
        enriquecimento_aceleracao[i] = EEPROM.read(endereco++);
    }

    // Ler os valores de tps_dot_escala (2 bytes cada)
    for (int i = 0; i < 5; i++) {
        tps_dot_escala[i] = EEPROM.read(endereco++);           // Byte menos significativo
        tps_dot_escala[i] |= EEPROM.read(endereco++) << 8;     // Byte mais significativo
    }

    // Ler os valores dos parâmetros restantes
    tipo_verificacao_aceleracao_rapida = EEPROM.read(endereco++);
    tps_mudanca_minima = EEPROM.read(endereco++);
    intervalo_tempo_aceleracao = EEPROM.read(endereco++);       // Byte menos significativo
    intervalo_tempo_aceleracao |= EEPROM.read(endereco++) << 8; // Byte mais significativo
    duracao_enriquecimento = EEPROM.read(endereco++);           // Byte menos significativo
    duracao_enriquecimento |= EEPROM.read(endereco++) << 8;     // Byte mais significativo
    rpm_minimo_enriquecimento = EEPROM.read(endereco++);       // Byte menos significativo
    rpm_minimo_enriquecimento |= EEPROM.read(endereco++) << 8; // Byte mais significativo
    rpm_maximo_enriquecimento = EEPROM.read(endereco++);       // Byte menos significativo
    rpm_maximo_enriquecimento |= EEPROM.read(endereco++) << 8; // Byte mais significativo    
    enriquecimento_desaceleracao = EEPROM.read(endereco++);
}



void ler_dados_eeprom(){
  // Leitura dos valores RPM da EEPROM
  int highByte;
  int lowByte;
for (int i = 0; i < 16; i++) {
    highByte = EEPROM.read(i*2+10); // Lê o byte mais significativo da posição i*2+10
    lowByte = EEPROM.read(i*2+11); // Lê o byte menos significativo da posição i*2+11
    vetor_rpm[i] = (highByte << 8) | lowByte; // Recria o valor original a partir dos dois bytes lidos  
}
// Leitura map e matrix ponto da EEPROM
for (int i = 0; i < 16; i++) {
  vetor_map_tps[i] = EEPROM.read(i+50);
  for (int j = 0; j < 16; j++) {
    matriz_avanco[i][j] = EEPROM.read(100 + i*16 + j);
  }
}

ler_dados_eeprom_tabela_ve_map_rpm();
ler_dados_eeprom_configuracao_injecao();
ler_dados_eeprom_configuracao_protecao();
ler_dados_eeprom_enriquecimento_aceleracao();
//leitura dos dados de configurações iniciais
tipo_ignicao = EEPROM.read(0*2+360); 
qtd_dente = EEPROM.read(1*2+360);
local_rodafonica = EEPROM.read(2*2+360);
qtd_dente_faltante = EEPROM.read(3*2+360);
highByte = EEPROM.read(4*2+360); // Lê o byte mais significativo 
lowByte = EEPROM.read(4*2+360+1); // Lê o byte menos significativo
grau_pms = (highByte << 8) | lowByte; 
grau_pms = grau_pms - 360; //volta os dados para valor original 
qtd_cilindro = EEPROM.read(5*2+360);
grau_entre_cada_cilindro = 360 / qtd_cilindro;
grau_cada_dente = 360 / qtd_dente;

//leitura dos dados de configurações de faisca 
referencia_leitura_ignicao = EEPROM.read(1*2+380); 
modo_ignicao = EEPROM.read(2*2+380);
grau_avanco_partida = EEPROM.read(3*2+380);
avanco_fixo = EEPROM.read(4*2+380);
grau_avanco_fixo = EEPROM.read(5*2+380);
tipo_sinal_bobina = EEPROM.read(6*2+380);

//leitura dos dados de configurações de dwell 
dwell_partida = EEPROM.read(1*2+400); 
dwell_funcionamento = EEPROM.read(2*2+400);

//leitura dos dados de calibração de ctl
referencia_temperatura_clt1 = EEPROM.read(410);
referencia_resistencia_clt1 = ler_valor_eeprom_2byte(412);
referencia_temperatura_clt2 = EEPROM.read(414);
referencia_resistencia_clt2 = ler_valor_eeprom_2byte(416); 

}

