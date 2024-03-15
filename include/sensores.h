float temperatura_clt(){
  int sensor_clt = analogRead(pino_sensor_clt);  // Lê o valor analógico
  float voltage_clt = sensor_clt * (5.0 / 1023.0);     // Converte o valor para tensão (0 a 5V)
  //float resistencia = 2000.0 * (5.0 / voltage_clt - 1.0);  // Resistência usando um resistor conhecido em ohms
  float resistencia_total = 2400.0;  // Substitua pelo valor real do seu uso
  float resistencia = resistencia_total * sensor_clt / 1023.0;
  //rpm = resistencia;
  float beta = calculateBeta(referencia_resistencia_clt1, referencia_temperatura_clt1, referencia_resistencia_clt2, referencia_temperatura_clt2);  
  //return calculateTemperature(resistencia, beta, referencia_resistencia_clt1, referencia_temperatura_clt1);
  int resultado_temperatura = calculateTemperature(resistencia, beta, referencia_resistencia_clt1, referencia_temperatura_clt1);

  if (resultado_temperatura > 250 || resultado_temperatura < 0) {
      return 250;
  } else {
      return resultado_temperatura;
  }
}
