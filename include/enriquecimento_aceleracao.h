
int tps_anterior = 0;   // Variável para armazenar o valor anterior do sensor de TPS
unsigned long previousMillis = 0;  // Variável para armazenar o tempo anterior de leitura do sensor
const int intervalo_tempo_aceleracao = 500; // Intervalo de tempo para calcular a taxa de mudança do TPS (em milissegundos)

  // Lê o valor do sensor de TPS
  tpsValue = analogRead(pinTPS);

  // Converte o valor lido do sensor de TPS para uma porcentagem (0% a 100%)
  float tpsPercentage = (tpsValue / 1023.0) * 100.0;

  // Calcula a taxa de mudança do TPS (TPSDot)
  if (tempo_atual - previousMillis >= intervalo_tempo_aceleracao) {
    // Calcula a diferença entre os valores atuais e anteriores do TPS em porcentagem
    float tpsDiffPercentage = abs(tpsPercentage - ((tps_anterior / 1023.0) * 100.0));
    // Calcula a taxa de mudança do TPS (TPSDot) em porcentagem por segundo
    float tpsDotPercentage = tpsDiffPercentage / (intervalo_tempo_aceleracao / 1000.0); // Converte o intervalo para segundos
    // Exibe a taxa de mudança do TPS (TPSDot) no monitor serial
    Serial.print("TPSDot: ");
    Serial.print(tpsDotPercentage);
    Serial.println("%/s");
    // Atualiza o valor anterior do TPS e o tempo de leitura
    tps_anterior = tpsValue;
    previousMillis = tempo_atual;
  }
