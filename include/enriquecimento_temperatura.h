static inline int limitar_enriquecimento_temperatura(int32_t valor) {
  if (valor < 0) {
    return 0;
  }
  if (valor > 250) {
    return 250;
  }
  return (int)valor;
}

// Tabela de 5 pontos (temperatura x enriquecimento % total).
// Aritmetica inteira, mesma interpolacao linear de antes.
int enriquecimento_temperatura(int temperatura) {
  if (temperatura <= vetor_temperatura_injecao[0]) {
    return limitar_enriquecimento_temperatura(vetor_enriquecimento_temperatura[0]);
  }

  for (int i = 1; i < 5; i++) {
    if (temperatura <= vetor_temperatura_injecao[i]) {
      int x0 = vetor_temperatura_injecao[i - 1];
      int x1 = vetor_temperatura_injecao[i];
      int y0 = vetor_enriquecimento_temperatura[i - 1];
      int y1 = vetor_enriquecimento_temperatura[i];

      if (x1 <= x0) {
        return limitar_enriquecimento_temperatura(y1);
      }

      int32_t largura = (int32_t)x1 - x0;
      int32_t numerador = (int32_t)(y1 - y0) * ((int32_t)temperatura - x0);
      int32_t ajuste = (numerador >= 0) ? (largura / 2) : -(largura / 2);
      return limitar_enriquecimento_temperatura(y0 + ((numerador + ajuste) / largura));
    }
  }

  return limitar_enriquecimento_temperatura(vetor_enriquecimento_temperatura[4]);
}
