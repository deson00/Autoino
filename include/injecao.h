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