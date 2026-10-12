void gravar_dados_eeprom_tabela_ignicao_map_rpm() {
    eeprom_cursor = 10; // Endereço base

    // 1. Gravar vetor_rpm (16 valores de 16 bits cada)
    for (int i = 0; i < 16; i++) {
        eeprom_gravar16_seguinte(vetor_rpm[i]);
    }
    // endereco agora = 42

    // 2. Gravar vetor_map_tps (16 valores de 8 bits cada)
    eeprom_cursor = 50; // Força endereço como no código original
    for (int i = 0; i < 16; i++) {
        eeprom_gravar8_seguinte(vetor_map_tps[i]);
    }
    // endereco agora = 66

    // 3. Gravar matriz_avanco (16x16 valores de 8 bits cada = 256 bytes)
    eeprom_cursor = 100; // Força endereço como no código original
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            eeprom_gravar8_seguinte(matriz_avanco[i][j]);
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
    // 16 bits em us (LSB primeiro, igual ao ler_16bits_eeprom). Os enderecos
    // 403 e 405 eram padding do layout antigo de 1 byte por dwell, entao dava
    // pra crescer pra 16 bits sem remapear nada. 406 marca o formato novo.
    EEPROM.update(402, dwell_partida_us & 0xFF);
    EEPROM.update(403, (dwell_partida_us >> 8) & 0xFF);
    EEPROM.update(404, dwell_funcionamento_us & 0xFF);
    EEPROM.update(405, (dwell_funcionamento_us >> 8) & 0xFF);
    EEPROM.update(406, DWELL_EEPROM_MARCADOR_US);
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
    eeprom_cursor = 500; // Endereço base

    // 1. Gravar vetor_rpm_ve (16 valores de 16 bits cada)
    for (int i = 0; i < 16; i++) {
        eeprom_gravar16_seguinte(vetor_rpm_ve[i]);
    }
    // endereco agora = 532

    // 2. Gravar vetor_map_tps_ve (16 valores de 8 bits cada)
    for (int i = 0; i < 16; i++) {
        eeprom_gravar8_seguinte(vetor_map_tps_ve[i]);
    }
    // endereco agora = 548

    // 3. Gravar matriz_ve (16x16 valores de 8 bits cada = 256 bytes)
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            eeprom_gravar8_seguinte(matriz_ve[i][j]);
        }
    }
    // endereco final = 804
}

void gravar_dados_eeprom_configuracao_partida() {
    // Bloco livre e valido tanto no ATmega328P quanto no ATmega2560.
    escrever_16bits_eeprom(820, rpm_partida);
    EEPROM.update(822, nivel_limpeza_afogamento);
    escrever_16bits_eeprom(823, atraso_injecao_inicial);
    escrever_16bits_eeprom(825, tempo_reducao_enriquecimento_partida);
    EEPROM.update(827, 0xA5);
}

void gravar_dados_eeprom_configuracao_marcha_lenta() {
    EEPROM.update(830, modo_marcha_lenta);
    EEPROM.update(831, temperatura_desligamento_marcha_lenta);
    EEPROM.update(832, histerese_marcha_lenta);
    EEPROM.update(833, pwm_marcha_lenta_frio);
    EEPROM.update(834, pwm_marcha_lenta_quente);
    escrever_16bits_eeprom(835, rpm_alvo_marcha_lenta);
    EEPROM.update(837, maximo_passos_marcha_lenta);
    EEPROM.update(838, inverter_direcao_marcha_lenta);
    EEPROM.update(839, 0xA6);
}


void gravar_dados_eeprom_configuracao_injecao(){
  eeprom_cursor = 900; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    eeprom_gravar8_seguinte(referencia_leitura_injecao & 0xFF); 
    eeprom_gravar8_seguinte(tipo_motor & 0xFF);
    eeprom_gravar8_seguinte(modo_injecao & 0xFF);
    eeprom_gravar8_seguinte(emparelhar_injetor & 0xFF);
    // deslocamento_motor (16 bits)
    eeprom_gravar16_seguinte(deslocamento_motor);
    eeprom_gravar8_seguinte(numero_cilindro_injecao);
    eeprom_gravar8_seguinte(numero_injetor);
    eeprom_gravar8_seguinte(numero_esguicho);
    // tamanho_injetor (16 bits)
    eeprom_gravar16_seguinte(tamanho_injetor);
    eeprom_gravar8_seguinte(tipo_acionamento_injetor);
    // tipo_combustivel (16 bits)
    eeprom_gravar16_seguinte(tipo_combustivel);
    // REQ_FUEL (16 bits)
    eeprom_gravar16_seguinte(REQ_FUEL);
    // dreq_fuel (16 bits)
    eeprom_gravar16_seguinte(dreq_fuel);
    eeprom_gravar8_seguinte(tipo_sonda_o2 ? 1 : 0);
    
}

void gravar_dados_eeprom_configuracao_iat() {
    EEPROM.update(420, referencia_temperatura_iat1 & 0xFF);
    EEPROM.update(422, referencia_resistencia_iat1 & 0xFF);
    EEPROM.update(423, (referencia_resistencia_iat1 >> 8) & 0xFF);
    EEPROM.update(424, referencia_temperatura_iat2 & 0xFF);
    EEPROM.update(426, referencia_resistencia_iat2 & 0xFF);
    EEPROM.update(427, (referencia_resistencia_iat2 >> 8) & 0xFF);
    EEPROM.update(428, 0xA5);
}

void gravar_dados_eeprom_parametros_injetor() {
    EEPROM.update(920, limite_injetor);
    escrever_16bits_eeprom(921, tempo_abertura_injetor);
    escrever_16bits_eeprom(923, grau_fechamento_injetor);
    EEPROM.update(925, acrescimo_injecao_partida);
    EEPROM.update(926, acrescimo_injecao_funcionamento);
    EEPROM.update(927, 0xA5); // Marca o bloco como inicializado.
}

void gravar_dados_eeprom_configuracao_protecao(){
  eeprom_cursor = 950; // Inicializa o endereço de memória
    // Gravar os valores divididos em bytes
    eeprom_gravar8_seguinte(tipo_protecao & 0xFF);
    // rpm_pre_corte (16 bits) 
    eeprom_gravar16_seguinte(rpm_pre_corte);
    eeprom_gravar8_seguinte(avanco_corte & 0xFF);
    eeprom_gravar8_seguinte(tempo_corte & 0xFF);
    // rpm_maximo_corte (16 bits)
    eeprom_gravar16_seguinte(rpm_maximo_corte);
    eeprom_gravar8_seguinte(numero_base_corte & 0xFF);
    eeprom_gravar8_seguinte(qtd_corte & 0xFF);
}
void gravar_dados_eeprom_enriquecimento_aceleracao() {
    eeprom_cursor = 970; // Inicializa o endereço de memória
    // enriquecimento_aceleracao (5 bytes de 1 byte cada)
    for (int i = 0; i < 5; i++) {
        eeprom_gravar8_seguinte(enriquecimento_aceleracao[i] & 0xFF);
    }
    // tps_dot_escala (5 valores de 2 bytes cada)
    for (int i = 0; i < 5; i++) {
        eeprom_gravar16_seguinte(tps_dot_escala[i]);
    }
    // Gravar os valores dos parâmetros restantes
    eeprom_gravar8_seguinte(tipo_verificacao_aceleracao_rapida & 0xFF);
    eeprom_gravar8_seguinte(tps_mudanca_minima);
    // intervalo_tempo_aceleracao (16 bits)
    eeprom_gravar16_seguinte(intervalo_tempo_aceleracao);
    // duracao_enriquecimento (16 bits)
    eeprom_gravar16_seguinte(duracao_enriquecimento);
    // rpm_minimo_enriquecimento (16 bits)
    eeprom_gravar16_seguinte(rpm_minimo_enriquecimento);
    // rpm_maximo_enriquecimento (16 bits)
    eeprom_gravar16_seguinte(rpm_maximo_enriquecimento);
    eeprom_gravar8_seguinte(enriquecimento_desaceleracao & 0xFF); // Grava o valor de desaceleração
}
void gravar_dados_eeprom_configuracao_tps() {
    eeprom_cursor = 1000; // Inicializa o endereço de memória
    // Gravar os valores dos parâmetros 
    // valor_tps_minimo (16 bits)
    eeprom_gravar16_seguinte(valor_tps_minimo);
    // valor_tps_maximo (16 bits)
    eeprom_gravar16_seguinte(valor_tps_maximo);
}
#include <EEPROM.h>

void gravar_dados_eeprom_configuracao_map() {
    eeprom_cursor = 1010; // Inicializa o endereço de memória
    // Gravar o valor do tipo de MAP (1 byte)
    eeprom_gravar8_seguinte(valor_map_tipo & 0xFF); // Grava o tipo de MAP
    // Gravar o valor mínimo do MAP (2 bytes)
    eeprom_gravar16_seguinte(valor_map_minimo);
    // Gravar o valor máximo do MAP (2 bytes)
    eeprom_gravar16_seguinte(valor_map_maximo);
}

// Tabela de offset por cilindro: 860..875 (8 x 16 bits) e a bandeira em 876.
// Fica no mesmo bloco livre que as tabelas de temperatura receberam - ver a
// nota abaixo sobre por que nada pode passar de 1023 nas placas de 328P.
void gravar_dados_eeprom_offset_evento() {
    eeprom_cursor = 860;
    for (byte i = 0; i < MAX_EVENTOS_AGENDAMENTO; i++) {
        eeprom_gravar16_seguinte((uint16_t)offset_evento[i]);
    }
    EEPROM.update(876, usar_offset_personalizado ? 1 : 0);
}

void gravar_dados_eeprom_enriquecimento_temperatura() {
    // Endereco 840, e nao 1020, porque a ATmega328P da Nano e da Uno tem 1024
    // bytes de EEPROM e o registrador de endereco e de 10 bits: escrever em
    // 1024 ou acima da a volta e cai no comeco. Nem o EEPROM.h do Arduino nem o
    // eeprom_write_byte da avr-libc mascaram isso.
    //
    // Com o bloco em 1020 os enderecos 1024..1029 caiam em 0..5, que nao sao
    // usados - passava despercebido. O bloco de avanco por temperatura, em
    // 1030, caia em 6..15 e corrompia vetor_rpm[0..2], os tres primeiros pontos
    // do eixo de rotacao da tabela de avanco. Na Mega, com 4096 bytes, nada
    // disso acontecia, e e nela que se desenvolve.
    //
    // 840..899 estava livre e cabem os dois blocos mais a tabela de offset.
    eeprom_cursor = 840;

    for (int i = 0; i < 5; i++) {
        eeprom_gravar8_seguinte(vetor_temperatura_injecao[i] & 0xFF);
    }
    for (int i = 0; i < 5; i++) {
        eeprom_gravar8_seguinte(vetor_enriquecimento_temperatura[i] & 0xFF);
    }

    // Endereco livre entre a configuracao do MAP (1010-1014) e os vetores.
    EEPROM.update(1015, usar_injecao_temperatura ? 1 : 0);
}

void gravar_dados_eeprom_avanco_temperatura() {
    // Ver a nota em gravar_dados_eeprom_enriquecimento_temperatura: este era o
    // bloco que corrompia o eixo de rotacao da tabela de avanco na 328P.
    eeprom_cursor = 850;

    for (int i = 0; i < 5; i++) {
        eeprom_gravar8_seguinte(vetor_temperatura[i] & 0xFF);
    }
    for (int i = 0; i < 5; i++) {
        eeprom_gravar8_seguinte(vetor_avanco_temperatura[i] & 0xFF);
    }

    // Segundo endereco livre reservado para a flag de avanco.
    EEPROM.update(1016, usar_avanco_temperatura ? 1 : 0);
}
