// Leitura compartilhada de sensor NTC (CLT/IAT usam a mesma conversao
// resistencia -> Beta -> Steinhart-Hart, so muda o pino e a calibracao).
#define NTC_RESISTENCIA_TOTAL 2400UL

int ler_temperatura_ntc(byte pino, int resistencia_ref1, int temperatura_ref1,
                        int resistencia_ref2, int temperatura_ref2) {
  int sensor = analogRead(pino);
  if (sensor <= 0) {
    return 250;
  }

  // Mesma conversao de antes (R = Rtotal * leitura / 1023), agora em inteiro.
  uint32_t resistencia = (NTC_RESISTENCIA_TOTAL * (uint32_t)sensor) / 1023UL;

  int resultado_temperatura = calcular_temperatura_ntc(resistencia,
                                                       resistencia_ref1, temperatura_ref1,
                                                       resistencia_ref2, temperatura_ref2);

  if (resultado_temperatura > 250 || resultado_temperatura < 0) {
    return 250;
  }
  return resultado_temperatura;
}

int temperatura_clt(){
  return ler_temperatura_ntc(pino_sensor_clt, referencia_resistencia_clt1, referencia_temperatura_clt1,
                              referencia_resistencia_clt2, referencia_temperatura_clt2);
}

int temperatura_iat(){
  return ler_temperatura_ntc(pino_sensor_iat, referencia_resistencia_iat1, referencia_temperatura_iat1,
                              referencia_resistencia_iat2, referencia_temperatura_iat2);
}
