
bool eeprom_configuracao_inicial_valida() {
    byte tipo_ignicao_eeprom = ler_8bits_eeprom(360);
    byte qtd_dente_eeprom = ler_8bits_eeprom(362);
    byte local_rodafonica_eeprom = ler_8bits_eeprom(364);
    byte qtd_dente_faltante_eeprom = ler_8bits_eeprom(366);
    uint16_t grau_pms_raw = ler_16bits_eeprom(368);
    byte qtd_cilindro_eeprom = ler_8bits_eeprom(370);

    if (tipo_ignicao_eeprom == 0xFF && qtd_dente_eeprom == 0xFF &&
        local_rodafonica_eeprom == 0xFF && qtd_dente_faltante_eeprom == 0xFF &&
        grau_pms_raw == 0xFFFF && qtd_cilindro_eeprom == 0xFF) {
        return false;
    }

    if ((tipo_ignicao_eeprom != 1 && tipo_ignicao_eeprom != 2) ||
        (local_rodafonica_eeprom != 1 && local_rodafonica_eeprom != 2) ||
        qtd_dente_eeprom == 0 || qtd_dente_eeprom == 0xFF ||
        qtd_cilindro_eeprom == 0 || qtd_cilindro_eeprom == 0xFF ||
        qtd_dente_faltante_eeprom == 0xFF ||
        grau_pms_raw < 360 || grau_pms_raw > 720) {
        return false;
    }

    return true;
}

// Tabela 16x16 com os dois eixos: 16 RPM de 16 bits, 16 MAP/TPS de 8 bits e
// a matriz de 256 bytes, cada bloco no seu endereco. Avanco e VE tinham cada
// uma a sua copia destes laços; uma so libera flash na Nano.
static void ler_tabela_16_eeprom(int endereco_rpm, int endereco_map, int endereco_matriz,
                                 int *rpm_eixo, int *map_eixo, byte matriz[16][16]) {
    eeprom_cursor = endereco_rpm;
    for (int i = 0; i < 16; i++) {
        rpm_eixo[i] = eeprom_ler16_seguinte();
    }
    eeprom_cursor = endereco_map;
    for (int i = 0; i < 16; i++) {
        map_eixo[i] = eeprom_ler8_seguinte();
    }
    eeprom_cursor = endereco_matriz;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            matriz[i][j] = eeprom_ler8_seguinte();
        }
    }
}

void ler_dados_eeprom_tabela_ignicao_map_rpm() {
    // RPM em 10 (16 bits), MAP/TPS em 50 e a matriz em 100, como no código original.
    ler_tabela_16_eeprom(10, 50, 100, vetor_rpm, vetor_map_tps, matriz_avanco);
}
void ler_dados_eeprom_tabela_ve_map_rpm() {
    // Blocos seguidos: RPM 500-531, MAP/TPS 532-547, matriz 548-803.
    ler_tabela_16_eeprom(500, 532, 548, vetor_rpm_ve, vetor_map_tps_ve, matriz_ve);
}

void ler_dados_eeprom_configuracao_partida() {
    if (EEPROM.read(827) != 0xA5) {
        return;
    }

    rpm_partida = constrain(ler_16bits_eeprom(820), 100, 2000);
    nivel_limpeza_afogamento = (byte)constrain(EEPROM.read(822), 0, 100);
    atraso_injecao_inicial = constrain(ler_16bits_eeprom(823), 0, 10000);
    tempo_reducao_enriquecimento_partida = constrain(ler_16bits_eeprom(825), 0, 600);
}

void ler_dados_eeprom_configuracao_marcha_lenta() {
    bool bloco_novo = EEPROM.read(839) == 0xA6;
    bool bloco_legado = EEPROM.read(835) == 0xA5;
    if (!bloco_novo && !bloco_legado) {
        return;
    }

    modo_marcha_lenta = (byte)constrain(EEPROM.read(830), 0, bloco_novo ? 3 : 2);
    temperatura_desligamento_marcha_lenta = (byte)constrain(EEPROM.read(831), 0, 120);
    histerese_marcha_lenta = (byte)constrain(EEPROM.read(832), 0, 20);
    pwm_marcha_lenta_frio = (byte)constrain(EEPROM.read(833), 0, 100);
    pwm_marcha_lenta_quente = (byte)constrain(EEPROM.read(834), 0, 100);

    if (bloco_novo) {
        rpm_alvo_marcha_lenta = constrain(ler_16bits_eeprom(835), 500, 2500);
        maximo_passos_marcha_lenta = (byte)constrain(EEPROM.read(837), 0, 250);
        inverter_direcao_marcha_lenta = EEPROM.read(838) ? 1 : 0;
    }
}

void ler_dados_eeprom_configuracao_injecao(){
  eeprom_cursor = 900; // Inicializa o endereço de memória

  // Ler os valores divididos em bytes
  referencia_leitura_injecao = eeprom_ler8_seguinte();
  tipo_motor = eeprom_ler8_seguinte();
  modo_injecao = eeprom_ler8_seguinte();
  emparelhar_injetor = eeprom_ler8_seguinte();
  
  // Para deslocamento_motor, combinamos os bytes lidos
  // deslocamento_motor (16 bits)
  deslocamento_motor = eeprom_ler16_seguinte();
  
  numero_cilindro_injecao = eeprom_ler8_seguinte();
  numero_injetor = eeprom_ler8_seguinte();
  numero_esguicho = eeprom_ler8_seguinte();
  
  // Para tamanho_injetor, combinamos os bytes lidos
  // tamanho_injetor (16 bits)
  tamanho_injetor = eeprom_ler16_seguinte();
  
  tipo_acionamento_injetor = eeprom_ler8_seguinte();
  // tipo_combustivel (16 bits)
  tipo_combustivel = eeprom_ler16_seguinte();
  
  // REQ_FUEL (16 bits)
  REQ_FUEL = eeprom_ler16_seguinte();
  
  // dreq_fuel (16 bits)
  dreq_fuel = eeprom_ler16_seguinte();

    byte tipo_sonda_eeprom = eeprom_ler8_seguinte();
    if (tipo_sonda_eeprom <= 1) {
        tipo_sonda_o2 = tipo_sonda_eeprom;
    } else {
        tipo_sonda_o2 = 1;
    }
}

void ler_dados_eeprom_parametros_injetor() {
    if (EEPROM.read(927) != 0xA5) {
        return;
    }

    limite_injetor = (byte)constrain(EEPROM.read(920), 0, 100);
    tempo_abertura_injetor = constrain(ler_16bits_eeprom(921), 0, 5000);
    grau_fechamento_injetor = constrain(ler_16bits_eeprom(923), 0, 360);
    acrescimo_injecao_partida = (byte)constrain(EEPROM.read(925), 0, 100);
    acrescimo_injecao_funcionamento = (byte)constrain(EEPROM.read(926), 0, 100);
}

void ler_dados_eeprom_configuracao_iat() {
    if (EEPROM.read(428) != 0xA5) {
        return;
    }

    referencia_temperatura_iat1 = ler_8bits_eeprom(420);
    referencia_resistencia_iat1 = ler_16bits_eeprom(422);
    referencia_temperatura_iat2 = ler_8bits_eeprom(424);
    referencia_resistencia_iat2 = ler_16bits_eeprom(426);
}


void ler_dados_eeprom_configuracao_protecao(){
  eeprom_cursor = 950; // Inicializa o endereço de memória
  // Ler os valores e atribuir às variáveis correspondentes
  tipo_protecao = eeprom_ler8_seguinte();
  // rpm_pre_corte (16 bits)
  rpm_pre_corte = eeprom_ler16_seguinte();
  avanco_corte = eeprom_ler8_seguinte();
  tempo_corte = eeprom_ler8_seguinte();
  // rpm_maximo_corte (16 bits)
  rpm_maximo_corte = eeprom_ler16_seguinte();
  numero_base_corte = eeprom_ler8_seguinte();
  qtd_corte = eeprom_ler8_seguinte();
}

void ler_dados_eeprom_enriquecimento_aceleracao() {
    eeprom_cursor = 970; // Inicializa o endereço de memória

    // Ler os valores de enriquecimento_aceleracao (1 byte cada)
    for (int i = 0; i < 5; i++) {
        enriquecimento_aceleracao[i] = eeprom_ler8_seguinte();
    }

    // Ler os valores de tps_dot_escala (2 bytes cada)
    for (int i = 0; i < 5; i++) {
      tps_dot_escala[i] = eeprom_ler16_seguinte();
    }

    // Ler os valores dos parâmetros restantes
    tipo_verificacao_aceleracao_rapida = eeprom_ler8_seguinte();
    tps_mudanca_minima = eeprom_ler8_seguinte();
        // Valores de 16 bits
    intervalo_tempo_aceleracao = eeprom_ler16_seguinte();
    duracao_enriquecimento = eeprom_ler16_seguinte();
    rpm_minimo_enriquecimento = eeprom_ler16_seguinte();
    rpm_maximo_enriquecimento = eeprom_ler16_seguinte();
    enriquecimento_desaceleracao = eeprom_ler8_seguinte();

}
void leitura_dados_eeprom_configuracao_tps() {
    eeprom_cursor = 1000; // Inicializa o endereço de memória
    // Ler os valores dos parâmetros
    valor_tps_minimo = eeprom_ler16_seguinte();
    
    valor_tps_maximo = eeprom_ler16_seguinte();
}
void ler_dados_eeprom_configuracao_map() {
    eeprom_cursor = 1010; // ENDEREÇO CORRIGIDO
    valor_map_tipo = eeprom_ler8_seguinte();
    valor_map_minimo = eeprom_ler16_seguinte();
    valor_map_maximo = eeprom_ler16_seguinte();
}

void ler_dados_eeprom_offset_evento() {
    // Bandeira diferente de 1 (inclusive 0xFF de memoria virgem) significa
    // tabela nao configurada: fica o preenchimento uniforme que
    // ler_dados_eeprom_configuracao_inicial acabou de fazer.
    if (EEPROM.read(876) != 1) {
        usar_offset_personalizado = false;
        return;
    }
    eeprom_cursor = 860;
    int lidos[MAX_EVENTOS_AGENDAMENTO];
    for (byte i = 0; i < MAX_EVENTOS_AGENDAMENTO; i++) {
        uint16_t bruto = eeprom_ler16_seguinte();
        // Fora de 0..359 e memoria corrompida, ou tabela gravada por uma versao
        // com outra geometria. Nesse caso volta para o uniforme, que sempre
        // roda, em vez de agendar faisca num angulo sem sentido fisico.
        if (bruto > 359) {
            usar_offset_personalizado = false;
            preencher_offset_evento_uniforme();
            return;
        }
        lidos[i] = (int)bruto;
    }
    // So publica depois que TODAS passaram: meia tabela boa e meia velha seria
    // pior que nenhuma.
    for (byte i = 0; i < MAX_EVENTOS_AGENDAMENTO; i++) {
        offset_evento[i] = lidos[i];
    }
    usar_offset_personalizado = true;
}

// Curva de 5 pontos (temperatura -> valor) gravada como 5 temperaturas e 5
// valores seguidos a partir de "endereco". Bloco todo 0xFF = nunca gravado:
// fica a curva padrao. Limita as temperaturas a 150, os valores a
// "limite_valor" e nao deixa a temperatura voltar para tras.
//
// O enriquecimento e o avanco por temperatura tinham cada um a sua copia
// desta leitura, identicas a nao ser pelo limite do valor; juntas aqui
// liberaram flash na Nano para a VE interpolada.
static void ler_curva_temperatura_eeprom(int endereco, byte limite_valor, byte *temperaturas, byte *valores) {
    eeprom_cursor = endereco;

    byte temperaturas_local[5];
    byte valores_local[5];

    for (int i = 0; i < 5; i++) {
        temperaturas_local[i] = eeprom_ler8_seguinte();
    }
    for (int i = 0; i < 5; i++) {
        valores_local[i] = eeprom_ler8_seguinte();
    }

    // Valida bloco vazio (EEPROM sem gravação).
    bool bloco_vazio = true;
    for (int i = 0; i < 5; i++) {
        if (temperaturas_local[i] != 0xFF || valores_local[i] != 0xFF) {
            bloco_vazio = false;
            break;
        }
    }
    if (bloco_vazio) {
        return;
    }

    // Aplica limites e monotonicidade mínima das temperaturas.
    for (int i = 0; i < 5; i++) {
        if (temperaturas_local[i] > 150) {
            temperaturas_local[i] = 150;
        }
        if (valores_local[i] > limite_valor) {
            valores_local[i] = limite_valor;
        }
    }
    for (int i = 1; i < 5; i++) {
        if (temperaturas_local[i] < temperaturas_local[i - 1]) {
            temperaturas_local[i] = temperaturas_local[i - 1];
        }
    }

    for (int i = 0; i < 5; i++) {
        temperaturas[i] = temperaturas_local[i];
        valores[i] = valores_local[i];
    }
}

void ler_dados_eeprom_enriquecimento_temperatura() {
    // 0xFF indica que a flag ainda nao foi gravada; nesse caso permanece desativada.
    usar_injecao_temperatura = EEPROM.read(1015) == 1 ? 1 : 0;
    // Ver a nota no gravador: 1020 estourava os 1024 bytes da 328P.
    ler_curva_temperatura_eeprom(840, 250, vetor_temperatura_injecao, vetor_enriquecimento_temperatura);
}

void ler_dados_eeprom_avanco_temperatura() {
    // 0xFF indica que a flag ainda nao foi gravada; nesse caso permanece desativada.
    usar_avanco_temperatura = EEPROM.read(1016) == 1 ? 1 : 0;
    // Ver a nota no gravador: 1030 caia em 6..15 e corrompia vetor_rpm.
    ler_curva_temperatura_eeprom(850, 10, vetor_temperatura, vetor_avanco_temperatura);
}

void ler_dados_eeprom_configuracao_inicial() {
    tipo_ignicao = ler_8bits_eeprom(360);
    qtd_dente = ler_8bits_eeprom(362);
    local_rodafonica = ler_8bits_eeprom(364);
    qtd_dente_faltante = ler_8bits_eeprom(366);
    grau_pms = ler_16bits_eeprom(368) - 360; // Simplesmente remove offset
    qtd_cilindro = ler_8bits_eeprom(370);

    if (qtd_cilindro < 1 || qtd_cilindro > 8) {
        qtd_cilindro = 1; // Fallback caso memoria corrompa
    }
    
    // Removidas as predições forçadas. O valor da tela sobrevive fielmente.
    grau_entre_cada_cilindro = calcular_grau_entre_cada_cilindro();
          if (!usar_offset_personalizado) preencher_offset_evento_uniforme();
    grau_cada_dente = 360 / qtd_dente;
}
void ler_dados_eeprom(){
    // Parametros do injetor possuem bloco proprio com marcador de validade (0xA5).
    // Carrega sempre para garantir persistencia, mesmo se a configuracao inicial estiver invalida.
    ler_dados_eeprom_parametros_injetor();
    ler_dados_eeprom_configuracao_partida();
    ler_dados_eeprom_configuracao_marcha_lenta();

    if (!eeprom_configuracao_inicial_valida()) {
        return;
    }

    ler_dados_eeprom_tabela_ignicao_map_rpm();
    ler_dados_eeprom_tabela_ve_map_rpm();
    ler_dados_eeprom_configuracao_injecao();
    ler_dados_eeprom_configuracao_iat();
    ler_dados_eeprom_configuracao_protecao();
    ler_dados_eeprom_enriquecimento_aceleracao();
    leitura_dados_eeprom_configuracao_tps();
    ler_dados_eeprom_configuracao_map();
    ler_dados_eeprom_enriquecimento_temperatura();
    ler_dados_eeprom_avanco_temperatura();
    ler_dados_eeprom_configuracao_inicial();
    // Depois da configuracao inicial de proposito: e la que o preenchimento
    // uniforme acontece, e a tabela gravada precisa vir por cima dele.
    ler_dados_eeprom_offset_evento();
    
    // Leitura dos dados de configurações de faisca 
    eeprom_cursor = 382;
    referencia_leitura_ignicao = ler_8bits_eeprom(eeprom_cursor); eeprom_cursor += 2;
    modo_ignicao = ler_8bits_eeprom(eeprom_cursor); eeprom_cursor += 2;
    grau_avanco_partida = ler_8bits_eeprom(eeprom_cursor); eeprom_cursor += 2;
    avanco_fixo = ler_8bits_eeprom(eeprom_cursor); eeprom_cursor += 2;
    grau_avanco_fixo = ler_8bits_eeprom(eeprom_cursor); eeprom_cursor += 2;
    tipo_sinal_bobina = ler_8bits_eeprom(eeprom_cursor);

    // Leitura dos dados de configurações de dwell
    // Com o marcador em 406, 402/404 sao 16 bits em us. Sem ele a EEPROM ainda
    // esta no formato antigo (1 byte em ms por dwell) - le como ms e converte,
    // senao um dwell de 3 ms viraria 3 us e o motor nao daria faisca nenhuma.
    if (ler_8bits_eeprom(406) == DWELL_EEPROM_MARCADOR_US) {
        dwell_partida_us = normalizar_dwell_us((long)ler_16bits_eeprom(402));
        dwell_funcionamento_us = normalizar_dwell_us((long)ler_16bits_eeprom(404));
    } else {
        dwell_partida_us = normalizar_dwell_us((long)ler_8bits_eeprom(402) * 1000L);
        dwell_funcionamento_us = normalizar_dwell_us((long)ler_8bits_eeprom(404) * 1000L);
    }

    // Leitura dos dados de calibração de CLT
    referencia_temperatura_clt1 = ler_8bits_eeprom(410);
    referencia_resistencia_clt1 = ler_16bits_eeprom(412);
    referencia_temperatura_clt2 = ler_8bits_eeprom(414);
    referencia_resistencia_clt2 = ler_16bits_eeprom(416); 

}

