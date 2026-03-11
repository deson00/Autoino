static inline byte limitar_avanco_temperatura(byte valor) {
  if (valor > 10) {
    return 10;
  }
  return valor;
}

// Retorna acrescimo de avanço em graus (0..10) via interpolacao linear em 5 pontos.
byte avanco_por_temperatura(float temperatura) {
  if (temperatura <= vetor_temperatura[0]) {
    return limitar_avanco_temperatura(vetor_avanco_temperatura[0]);
  }

  for (int i = 1; i < 5; i++) {
    if (temperatura <= vetor_temperatura[i]) {
      float x0 = (float)vetor_temperatura[i - 1];
      float x1 = (float)vetor_temperatura[i];
      float y0 = (float)vetor_avanco_temperatura[i - 1];
      float y1 = (float)vetor_avanco_temperatura[i];

      if (x1 <= x0) {
        return limitar_avanco_temperatura((byte)y1);
      }

      float t = (temperatura - x0) / (x1 - x0);
      float y = y0 + ((y1 - y0) * t);
      if (y < 0.0f) {
        y = 0.0f;
      }
      if (y > 10.0f) {
        y = 10.0f;
      }
      return (byte)(y + 0.5f);
    }
  }

  return limitar_avanco_temperatura(vetor_avanco_temperatura[4]);
}
