
float enriquecimento_gama(float tempo_base_injecao, float correcao_aquecimento, float correcao_O2, float correcao_temperatura_ar, float correcao_barometrica) {
    //valores maiores que 100 adiciona e valores menor que 100 retira
    // Calcula o fator de correção geral multiplicando os fatores individuais
  float fator_correcao_total = (correcao_aquecimento / 100.0) * (correcao_O2 / 100.0) * (correcao_temperatura_ar / 100.0) * (correcao_barometrica / 100.0);

  // Calcula o tempo de enriquecimento gama
  float tempo_enriquecimento_gama = tempo_base_injecao * fator_correcao_total;

  return tempo_enriquecimento_gama;
}