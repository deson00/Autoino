volatile bool sincronizado_estavel = false;
volatile uint8_t contador_sync_consecutivas = 0;
volatile uint32_t ultimo_gap_detectado = 0;
volatile uint32_t timeout_ultima_atividade = 0;
volatile uint32_t ultimo_tempo_interrupcao = 0;
volatile uint16_t intervalo_dente_filtrado = 0;

#define MIN_INTERVALO_DENTE_US 50        // Filtro anti-bounce mínimo
#define MAX_INTERVALO_DENTE_US 500000     // Máximo para RPM muito baixo (~120 RPM)
#define FATOR_GAP_MINIMO 1.5             // Gap mínimo para detectar falha (1.5x)
#define FATOR_GAP_MAXIMO 5.0             // Gap máximo válido (5x)
#define MIN_SYNC_REVOLUCOES 1            // Revoluções mínimas para considerar sincronizado
#define TIMEOUT_SINCRONIA_MS 2000        // Timeout para perder sincronização

// Filtro de média móvel para estabilizar intervalos
volatile uint32_t buffer_intervalos[4] = {0, 0, 0, 0};
volatile uint8_t indice_buffer = 0;

// ========== FUNÇÕES AUXILIARES ==========

// Filtro de média móvel simples para suavizar leituras
uint32_t aplicar_filtro_media_movel(uint32_t novo_intervalo) {
    buffer_intervalos[indice_buffer] = novo_intervalo;
    indice_buffer = (indice_buffer + 1) & 0x03; // % 4 otimizado
    
    // Calcula média (divisão por 4 = shift right 2)
    return (buffer_intervalos[0] + buffer_intervalos[1] + 
            buffer_intervalos[2] + buffer_intervalos[3]) >> 2;
}

// Validação robusta de intervalo de dente
bool intervalo_valido(uint32_t intervalo) {
    return (intervalo >= MIN_INTERVALO_DENTE_US && 
            intervalo <= MAX_INTERVALO_DENTE_US);
}

// Detecção de gap mais robusta
bool detectar_gap(uint32_t intervalo_atual, uint32_t intervalo_anterior) {
    if (intervalo_anterior == 0) return false;
    
    float razao = (float)intervalo_atual / intervalo_anterior;
    return (razao >= FATOR_GAP_MINIMO && razao <= FATOR_GAP_MAXIMO);
}

// ========== DECODER PRINCIPAL MELHORADO ==========
void decoder_roda_fonica_otimizado() {
    uint32_t tempo_agora = micros();
    
    // FILTRO 1: Anti-bounce adaptativo baseado no RPM atual
    uint32_t min_intervalo = (rpm > 1000) ? MIN_INTERVALO_DENTE_US : MIN_INTERVALO_DENTE_US * 2;
    if ((tempo_agora - ultimo_tempo_interrupcao) < min_intervalo) {
        return;
    }
    
    // FILTRO 2: Timeout de atividade (detecta motor parado)
    timeout_ultima_atividade = tempo_agora;
    
    // Atualiza contadores e tempos - USANDO SUAS VARIÁVEIS
    qtd_leitura++;
    tempo_atual = tempo_agora;
    intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);
    ultimo_tempo_interrupcao = tempo_agora;
    
    // FILTRO 3: Validação de intervalo
    if (!intervalo_valido(intervalo_tempo_entre_dente)) {
        // Intervalo inválido - não processa mas mantém tempo anterior
        tempo_anterior = tempo_atual;
        return;
    }
    
    // FILTRO 4: Aplica média móvel para suavizar (opcional em RPM estável)
    if (rpm > 500 && sincronizado_estavel) {
        intervalo_dente_filtrado = aplicar_filtro_media_movel(intervalo_tempo_entre_dente);
    } else {
        intervalo_dente_filtrado = intervalo_tempo_entre_dente;
    }
    
    //Cálculo de falha
    uint32_t intervalo_referencia = tempo_dente_anterior[leitura];
    verifica_falha = intervalo_referencia + (intervalo_referencia >> 1); // 1.5x mais eficiente
    
    //Inicialização
    if (inicia_tempo_sensor_roda_fonica) {
        tempo_anterior = tempo_atual;
        tempo_dente_anterior[0] = intervalo_tempo_entre_dente;
        tempo_dente_anterior[1] = intervalo_tempo_entre_dente;
        inicia_tempo_sensor_roda_fonica = 0;
        
        // Reset do estado de sincronização
        sincronizado_estavel = false;
        contador_sync_consecutivas = 0;
    }
    
    //Alternância de buffer
    if (leitura == 0) {
        leitura = 1;
        tempo_dente_anterior[0] = intervalo_dente_filtrado;
    } else {
        leitura = 0;
        tempo_dente_anterior[1] = intervalo_dente_filtrado;
    }
    
    // DETECÇÃO DE GAP
    bool gap_detectado = detectar_gap(intervalo_tempo_entre_dente, intervalo_referencia);
    
    if (gap_detectado || (verifica_falha < intervalo_tempo_entre_dente)) {
        ultimo_gap_detectado = tempo_atual;
        
        //Controle de volta completa
        if (qtd_voltas == 1) {
            tempo_final_volta_completa = tempo_atual;
            tempo_total_volta_completa = (tempo_final_volta_completa - tempo_inicio_volta_completa);
            qtd_voltas = 0;
            
            //Validação do tempo de volta
            if (tempo_total_volta_completa > 0) {
                // Calcula RPM para validação
                uint32_t rpm_calculado = 60000000UL / tempo_total_volta_completa;
                if (rpm_calculado < 10000) { // RPM razoável (< 10k)
                    rpm = rpm_calculado;
                }
            }
        }
        if (qtd_voltas == 0) {
            tempo_inicio_volta_completa = tempo_atual;
            qtd_voltas = 1;
        }
        
        qtd_revolucoes++;
        
        //Cálculo de tempo por grau
        if (tempo_total_volta_completa > 0) {
            tempo_cada_grau = tempo_total_volta_completa * 0.00277777778; // /360
        }
        
        // Reset de posição no gap
        posicao_atual_sensor = 0;
        
        // CONTROLE DE SINCRONIZAÇÃO
        uint16_t dentes_esperados = qtd_dente - qtd_dente_faltante;
        if (qtd_leitura == dentes_esperados) {
            // Sincronização correta
            revolucoes_sincronizada++;
            contador_sync_consecutivas++;
            
            // Considera sincronizado após revoluções consecutivas
            if (contador_sync_consecutivas >= MIN_SYNC_REVOLUCOES) {
                sincronizado_estavel = true;
            }
        } else {
            // Perda de sincronização
            if (++qtd_perda_sincronia >= 255) {
                qtd_perda_sincronia = 0;
            }
            contador_sync_consecutivas = 0;
            sincronizado_estavel = false;
        }
        
        qtd_leitura = 0;
        
        //Reset de flags para ignição/injeção
        if (tipo_ignicao_sequencial == 0) {
            tempo_atual_proxima_ignicao[0] = tempo_atual;
            ign_acionado[0] = false;
            captura_dwell[0] = false;
            tempo_atual_proxima_injecao[0] = tempo_atual;
            inj_acionado[0] = false;
            captura_req_fuel[0] = false;
        }
        
    } else {
        //Cálculo alternativo em baixo RPM
        if (rpm < rpm_partida && qtd_dente > 0) {
            tempo_cada_grau = intervalo_tempo_entre_dente / (360 / qtd_dente);
        }
    }
    
    // Atualização de posição com wrap-around 
    posicao_atual_sensor += grau_cada_dente;
    if (posicao_atual_sensor >= 360) {
        posicao_atual_sensor -= 360;
    }
    
    //Atualização do tempo anterior
    tempo_anterior = tempo_atual;
}

// ========== FUNÇÕES DE INTERRUPÇÃO ==========
// Função de interrupção otimizada
void leitor_sensor_roda_fonica() {
    decoder_roda_fonica_otimizado();
}

// ========== MONITORAMENTO E DIAGNÓSTICO ==========

// Verifica se perdeu sincronização por timeout
void verificar_timeout_sincronia() {
    uint32_t agora = millis();
    if (sincronizado_estavel && 
        (agora - (timeout_ultima_atividade / 1000)) > TIMEOUT_SINCRONIA_MS) {
        
        // Perdeu sincronização por timeout
        sincronizado_estavel = false;
        contador_sync_consecutivas = 0;
    }
}

// Debug avançado usando suas variáveis
void debug_roda_fonica_avancado() {
    static unsigned long ultimo_debug = 0;
    
    if (millis() - ultimo_debug > 500) { // Debug mais frequente
        ultimo_debug = millis();
        
        Serial.print("RPM: "); Serial.print(rpm);
        Serial.print(" | Pos: "); Serial.print(posicao_atual_sensor);
        Serial.print("° | Sync: "); 
        Serial.print(sincronizado_estavel ? "OK" : "NO");
        Serial.print(" ("); Serial.print(revolucoes_sincronizada); Serial.print(")");
        Serial.print(" | Perdas: "); Serial.print(qtd_perda_sincronia);
        Serial.print(" | ΔT: "); Serial.print(intervalo_tempo_entre_dente);
        Serial.print("µs | Leit: "); Serial.println(qtd_leitura);
        
        // Verifica timeout
        verificar_timeout_sincronia();
    }
}


// ========== INICIALIZAÇÃO ==========
void inicializar_decoder_otimizado() {
    // Reset das suas variáveis existentes
    qtd_leitura = 0;
    posicao_atual_sensor = 0;
    inicia_tempo_sensor_roda_fonica = 1;
    revolucoes_sincronizada = 0;
    qtd_perda_sincronia = 0;
    
    // Reset das melhorias
    sincronizado_estavel = false;
    contador_sync_consecutivas = 0;
    ultimo_gap_detectado = 0;
    timeout_ultima_atividade = millis() * 1000;
    
    // Limpa buffer de filtro
    for (int i = 0; i < 4; i++) {
        buffer_intervalos[i] = 0;
    }
    indice_buffer = 0;
}

// // ========== DIAGNÓSTICO COMPLETO ==========
// void diagnostico_decoder() {
//     Serial.println("\n=== DIAGNÓSTICO DECODER ===");
//     Serial.print("Configuração: "); Serial.print(qtd_dente); 
//     Serial.print("-"); Serial.println(qtd_dente_faltante);
//     Serial.print("Estado Sincronização: ");
//     Serial.println(sincronizado_estavel ? "ESTÁVEL" : "INSTÁVEL");
//     Serial.print("Revoluções Sincronizadas: "); Serial.println(revolucoes_sincronizada);
//     Serial.print("Perdas de Sincronia: "); Serial.println(qtd_perda_sincronia);
//     Serial.print("RPM Atual: "); Serial.println(rpm);
//     Serial.print("Posição Atual: "); 
//     Serial.print(posicao_atual_sensor); 
//     Serial.println("°");
//     Serial.print("Última Atividade: "); 
//     Serial.print((millis() * 1000 - timeout_ultima_atividade) / 1000); 
//     Serial.println(" ms atrás");
//     Serial.print("Cilindros: "); Serial.println(qtd_cilindro);
//     Serial.print("Tipo Ignição: "); 
//     Serial.println(tipo_ignicao_sequencial ? "Sequencial" : "Semi-Sequencial");
//     Serial.println("===========================\n");
// }
