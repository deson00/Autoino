float enriquecimento_temperatura(float temperatura, float temperatura_trabalho, float correcao_maxima) {
  if (temperatura >= temperatura_trabalho) {
    return 100.0; // 0% de correção
  } else if (temperatura <= 0.0) {
    return 100.0 + correcao_maxima; // 50% de enriquecimento
  } else {
    // Interpolação linear entre a correção máxima e 0%
    float fator_correcao = correcao_maxima * (1.0 - (temperatura / temperatura_trabalho));
    return 100.0 + fator_correcao;
  }
}