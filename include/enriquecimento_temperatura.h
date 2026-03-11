static inline float limitar_enriquecimento_temperatura(float valor) {
  if (valor < 0.0f) {
    return 0.0f;
  }
  if (valor > 250.0f) {
    return 250.0f;
  }
  return valor;
}

// Compatibilidade com o método linear legado.
float enriquecimento_temperatura(float temperatura, float temperatura_trabalho, float correcao_maxima) {
  if (temperatura >= temperatura_trabalho) {
    return 100.0f;
  }
  if (temperatura <= 0.0f) {
    return limitar_enriquecimento_temperatura(100.0f + correcao_maxima);
  }

  float fator_correcao = correcao_maxima * (1.0f - (temperatura / temperatura_trabalho));
  return limitar_enriquecimento_temperatura(100.0f + fator_correcao);
}

// Novo método: tabela de 5 pontos (temperatura x enriquecimento % total).
float enriquecimento_temperatura(float temperatura) {
  if (temperatura <= vetor_temperatura_injecao[0]) {
    return limitar_enriquecimento_temperatura((float)vetor_enriquecimento_temperatura[0]);
  }

  for (int i = 1; i < 5; i++) {
    if (temperatura <= vetor_temperatura_injecao[i]) {
      float x0 = (float)vetor_temperatura_injecao[i - 1];
      float x1 = (float)vetor_temperatura_injecao[i];
      float y0 = (float)vetor_enriquecimento_temperatura[i - 1];
      float y1 = (float)vetor_enriquecimento_temperatura[i];

      if (x1 <= x0) {
        return limitar_enriquecimento_temperatura(y1);
      }

      float t = (temperatura - x0) / (x1 - x0);
      return limitar_enriquecimento_temperatura(y0 + ((y1 - y0) * t));
    }
  }

  return limitar_enriquecimento_temperatura((float)vetor_enriquecimento_temperatura[4]);
}