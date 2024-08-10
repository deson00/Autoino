// Função para calcular a largura do pulso de injeção
float tempo_pulso_ve(float REQ_FUEL, float MAP, float VE) {
    const float tempo_pulso_ve = REQ_FUEL * (VE / 100.0);
    return tempo_pulso_ve * 1000ul;
}
// Função para calcular a largura do pulso enriquecimento
float tempo_enriquecimento_gama(float valor_referencia, float correcao_aquecimento, float correcao_O2, float correcao_temperatura_ar, float correcao_barometrica) {
    const float REQ_FUEL_CONSTANT = 100; 
    const float enriquecimento_gama = (correcao_aquecimento / REQ_FUEL_CONSTANT) * (correcao_O2 / REQ_FUEL_CONSTANT) *
                               (correcao_temperatura_ar / REQ_FUEL_CONSTANT) * (correcao_barometrica / REQ_FUEL_CONSTANT);
 
    return valor_referencia * enriquecimento_gama;
}

void configurarInjecao(int i, int ajuste_pms) {
    tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
}

void iniciarInjecao(int i) {
    if (tipo_acionamento_injetor == 1) {
        for (int j = 0; j < numero_injetor; j++) {
            digitalWrite(injecao_pins[j], HIGH);
        }
    } else {
        digitalWrite(injecao_pins[i], HIGH);
    }
    captura_req_fuel[i] = true;
    tempo_percorrido_inj[i] = tempo_atual;
    tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i];
    inj_acionado[i] = true;
    inj_acionado[(i + 1) % qtd_cilindro] = false;
    captura_req_fuel[(i + 1) % qtd_cilindro] = false;
}


void pararInjecao(int i) {
    captura_req_fuel[i] = false;
    if (tipo_acionamento_injetor == 1) {
        for (int j = 0; j < numero_injetor; j++) {
            digitalWrite(injecao_pins[j], LOW);
        }
    } else {
        digitalWrite(injecao_pins[i], LOW);
    }
}
