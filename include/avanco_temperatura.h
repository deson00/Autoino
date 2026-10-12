static inline byte limitar_avanco_temperatura(int valor) {
  if (valor < 0) {
    return 0;
  }
  if (valor > 10) {
    return 10;
  }
  return (byte)valor;
}

// Retorna acrescimo de avanço em graus (0..10) via interpolacao linear em 5 pontos.
// Aritmetica inteira: a interpolacao y0 + (y1-y0)*(t-x0)/(x1-x0) e feita com
// uma multiplicacao antes da divisao, preservando a precisao sem ponto
// flutuante. O +/- meio passo antes de dividir reproduz o arredondamento do
// codigo anterior.
byte avanco_por_temperatura(int temperatura) {
  if (temperatura <= vetor_temperatura[0]) {
    return limitar_avanco_temperatura(vetor_avanco_temperatura[0]);
  }

  for (int i = 1; i < 5; i++) {
    if (temperatura <= vetor_temperatura[i]) {
      int x0 = vetor_temperatura[i - 1];
      int x1 = vetor_temperatura[i];
      int y0 = vetor_avanco_temperatura[i - 1];
      int y1 = vetor_avanco_temperatura[i];

      if (x1 <= x0) {
        return limitar_avanco_temperatura(y1);
      }

      int32_t largura = (int32_t)x1 - x0;
      int32_t numerador = (int32_t)(y1 - y0) * ((int32_t)temperatura - x0);
      // arredonda para o inteiro mais proximo, respeitando o sinal
      int32_t ajuste = (numerador >= 0) ? (largura / 2) : -(largura / 2);
      int32_t y = y0 + ((numerador + ajuste) / largura);
      return limitar_avanco_temperatura((int)y);
    }
  }

  return limitar_avanco_temperatura(vetor_avanco_temperatura[4]);
}
