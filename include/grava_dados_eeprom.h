void gravar_dados_eeprom_tabela_ignicao_map_rpm() {
    int endereco = 10; // Endereço base

    // 1. Gravar vetor_rpm (16 valores de 16 bits cada)
    for (int i = 0; i < 16; i++) {
        escrever_16bits_eeprom(endereco, vetor_rpm[i]);
        endereco += 2;
    }
    // endereco agora = 42

    // 2. Gravar vetor_map_tps (16 valores de 8 bits cada)
    endereco = 50; // Força endereço como no código original
    for (int i = 0; i < 16; i++) {
        escrever_8bits_eeprom(endereco, vetor_map_tps[i]);
        endereco++;
    }
    // endereco agora = 66

    // 3. Gravar matriz_avanco (16x16 valores de 8 bits cada = 256 bytes)
    endereco = 100; // Força endereço como no código original
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            escrever_8bits_eeprom(endereco, matriz_avanco[i][j]);
            endereco++;
        }
    }
    // endereco final = 356
}

void gravar_dados_eeprom_configuracao_inicial() {
    escrever_8bits_eeprom(360, tipo_ignicao);
    escrever_8bits_eeprom(362, qtd_dente);
    escrever_8bits_eeprom(364, local_rodafonica);
    escrever_8bits_eeprom(366, qtd_dente_faltante);
    escrever_16bits_eeprom(368, grau_pms + 360); // Simplesmente adiciona offset
    escrever_8bits_eeprom(370, qtd_cilindro);
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
    int endereco = 500; // Endereço base

    // 1. Gravar vetor_rpm_ve (16 valores de 16 bits cada)
    for (int i = 0; i < 16; i++) {
        escrever_16bits_eeprom(endereco, vetor_rpm_ve[i]);
        endereco += 2;
    }
    // endereco agora = 532

    // 2. Gravar vetor_map_tps_ve (16 valores de 8 bits cada)
    for (int i = 0; i < 16; i++) {
        escrever_8bits_eeprom(endereco, vetor_map_tps_ve[i]);
        endereco++;
    }
    // endereco agora = 548

    // 3. Gravar matriz_ve (16x16 valores de 8 bits cada = 256 bytes)
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            escrever_8bits_eeprom(endereco, matriz_ve[i][j]);
            endereco++;
        }
    }
    // endereco final = 804
}


void gravar_dados_eeprom_configuracao_injecao(){
  int endereco =  900; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    EEPROM.update(endereco++, referencia_leitura_injecao & 0xFF); 
    EEPROM.update(endereco++, tipo_motor & 0xFF);
    EEPROM.update(endereco++, modo_injecao & 0xFF);
    EEPROM.update(endereco++, emparelhar_injetor & 0xFF);
    // deslocamento_motor (16 bits)
    EEPROM.update(endereco++, deslocamento_motor & 0xFF);
    EEPROM.update(endereco++, (deslocamento_motor >> 8) & 0xFF);
    EEPROM.update(endereco++, numero_cilindro_injecao);
    EEPROM.update(endereco++, numero_injetor);
    EEPROM.update(endereco++, numero_esguicho);
    // tamanho_injetor (16 bits)
    EEPROM.update(endereco++, tamanho_injetor & 0xFF);
    EEPROM.update(endereco++, (tamanho_injetor >> 8) & 0xFF);
    EEPROM.update(endereco++, tipo_acionamento_injetor);
    // tipo_combustivel (16 bits)
    EEPROM.update(endereco++, tipo_combustivel & 0xFF);
    EEPROM.update(endereco++, (tipo_combustivel >> 8) & 0xFF);
    // REQ_FUEL (16 bits)
    EEPROM.update(endereco++, REQ_FUEL & 0xFF);
    EEPROM.update(endereco++, (REQ_FUEL >> 8) & 0xFF);
    // dreq_fuel (16 bits)
    EEPROM.update(endereco++, (dreq_fuel & 0xFF));
    EEPROM.update(endereco++, (dreq_fuel >> 8) & 0xFF);
    
}

void gravar_dados_eeprom_configuracao_protecao(){
  int endereco =  950; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    EEPROM.update(endereco++, tipo_protecao & 0xFF);
    // rpm_pre_corte (16 bits) 
    EEPROM.update(endereco++, rpm_pre_corte & 0xFF);
    EEPROM.update(endereco++, (rpm_pre_corte >> 8) & 0xFF);
    EEPROM.update(endereco++, avanco_corte & 0xFF);
    EEPROM.update(endereco++, tempo_corte & 0xFF);
    // rpm_maximo_corte (16 bits)
    EEPROM.update(endereco++, rpm_maximo_corte & 0xFF);
    EEPROM.update(endereco++, (rpm_maximo_corte >> 8) & 0xFF);
    EEPROM.update(endereco++, numero_base_corte & 0xFF);
    EEPROM.update(endereco++, qtd_corte & 0xFF);
}
void gravar_dados_eeprom_enriquecimento_aceleracao() {
    int endereco = 970; // Inicializa o endereço de memória
    // enriquecimento_aceleracao (5 bytes de 1 byte cada)
    for (int i = 0; i < 5; i++) {
        EEPROM.update(endereco++, enriquecimento_aceleracao[i] & 0xFF);
    }
    // tps_dot_escala (5 valores de 2 bytes cada)
    for (int i = 0; i < 5; i++) {
        EEPROM.update(endereco++, tps_dot_escala[i] & 0xFF);          // Byte menos significativo
        EEPROM.update(endereco++, (tps_dot_escala[i] >> 8) & 0xFF);   // Byte mais significativo
    }
    // Gravar os valores dos parâmetros restantes
    EEPROM.update(endereco++, tipo_verificacao_aceleracao_rapida & 0xFF);
    EEPROM.update(endereco++, tps_mudanca_minima);
    // intervalo_tempo_aceleracao (16 bits)
    EEPROM.update(endereco++, intervalo_tempo_aceleracao & 0xFF);        // Byte menos significativo
    EEPROM.update(endereco++, (intervalo_tempo_aceleracao >> 8) & 0xFF); // Byte mais significativo
    // duracao_enriquecimento (16 bits)
    EEPROM.update(endereco++, duracao_enriquecimento & 0xFF);        // Byte menos significativo
    EEPROM.update(endereco++, (duracao_enriquecimento >> 8) & 0xFF); // Byte mais significativo
    // rpm_minimo_enriquecimento (16 bits)
    EEPROM.update(endereco++, rpm_minimo_enriquecimento & 0xFF);       // Byte menos significativo
    EEPROM.update(endereco++, (rpm_minimo_enriquecimento >> 8) & 0xFF); // Byte mais significativo
    // rpm_maximo_enriquecimento (16 bits)
    EEPROM.update(endereco++, rpm_maximo_enriquecimento & 0xFF);       // Byte menos significativo
    EEPROM.update(endereco++, (rpm_maximo_enriquecimento >> 8) & 0xFF); // Byte mais significativo
    EEPROM.update(endereco++, enriquecimento_desaceleracao & 0xFF); // Grava o valor de desaceleração
}
void gravar_dados_eeprom_configuracao_tps() {
    int endereco = 1000; // Inicializa o endereço de memória
    // Gravar os valores dos parâmetros 
    // valor_tps_minimo (16 bits)
    EEPROM.update(endereco++, valor_tps_minimo & 0xFF);        // Byte menos significativo
    EEPROM.update(endereco++, (valor_tps_minimo >> 8) & 0xFF); // Byte mais significativo
    // valor_tps_maximo (16 bits)
    EEPROM.update(endereco++, valor_tps_maximo & 0xFF);        // Byte menos significativo
    EEPROM.update(endereco++, (valor_tps_maximo >> 8) & 0xFF); // Byte mais significativo
}
#include <EEPROM.h>

void gravar_dados_eeprom_configuracao_map() {
    int endereco = 1010; // Inicializa o endereço de memória
    // Gravar o valor do tipo de MAP (1 byte)
    EEPROM.update(endereco++, valor_map_tipo & 0xFF); // Grava o tipo de MAP
    // Gravar o valor mínimo do MAP (2 bytes)
    EEPROM.update(endereco++, valor_map_minimo & 0xFF);        // Byte menos significativo
    EEPROM.update(endereco++, (valor_map_minimo >> 8) & 0xFF); // Byte mais significativo
    // Gravar o valor máximo do MAP (2 bytes)
    EEPROM.update(endereco++, valor_map_maximo & 0xFF);        // Byte menos significativo
    EEPROM.update(endereco++, (valor_map_maximo >> 8) & 0xFF); // Byte mais significativo
}