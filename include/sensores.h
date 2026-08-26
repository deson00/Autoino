// Leitura compartilhada de sensor NTC (CLT/IAT usam a mesma conversao
// resistencia -> Beta -> Steinhart-Hart, so muda o pino e a calibracao).
float ler_temperatura_ntc(byte pino, int resistencia_ref1, int temperatura_ref1,
                           int resistencia_ref2, int temperatura_ref2) {
  int sensor = analogRead(pino);
  float resistencia_total = 2400.0;  // Substitua pelo valor real do seu uso
  float resistencia = resistencia_total * sensor / 1023.0;
  float beta = calculateBeta(resistencia_ref1, temperatura_ref1, resistencia_ref2, temperatura_ref2);
  int resultado_temperatura = calculateTemperature(resistencia, beta, resistencia_ref1, temperatura_ref1);

  if (resultado_temperatura > 250 || resultado_temperatura < 0) {
    return 250;
  }
  return resultado_temperatura;
}

float temperatura_clt(){
  return ler_temperatura_ntc(pino_sensor_clt, referencia_resistencia_clt1, referencia_temperatura_clt1,
                              referencia_resistencia_clt2, referencia_temperatura_clt2);
}

float temperatura_iat(){
  return ler_temperatura_ntc(pino_sensor_iat, referencia_resistencia_iat1, referencia_temperatura_iat1,
                              referencia_resistencia_iat2, referencia_temperatura_iat2);
}
