void sendSerialString(const char* str) {
  while (*str) {
    Serial.write(*str++);
  }
}

void sendSerialInt(int value) {
  char buffer[10]; // Buffer para converter int para string
  itoa(value, buffer, 10); // Converter int para string
  sendSerialString(buffer); // Enviar a string resultante
}

void ler_dados_memoria() {
  // Enviar o identificador de comando
  Serial.write(';');
  Serial.write('a');
  Serial.write(',');

  // Enviar os valores do vetor_map_tps
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_map_tps[i]);
    if (i < 16) {
      Serial.write(',');
    }
  }
  Serial.write(';');

  Serial.write('b');
  Serial.write(',');
  // vetor rpm
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_rpm[i]);
    if (i < 16) {
      Serial.write(',');
    }
  }
  Serial.write(';');

  // transforma matriz em vetor
  Serial.write('c');
  Serial.write(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      sendSerialInt(matriz_avanco[i][j]);
      Serial.write(',');
    }
  }
  Serial.write(';');

  // dados configuração inicial
  Serial.write('g');
  Serial.write(',');
  sendSerialInt(tipo_ignicao);
  Serial.write(',');
  sendSerialInt(qtd_dente);
  Serial.write(',');
  sendSerialInt(local_rodafonica);
  Serial.write(',');
  sendSerialInt(qtd_dente_faltante);
  Serial.write(',');
  sendSerialInt(grau_pms);
  Serial.write(',');
  sendSerialInt(qtd_cilindro * local_rodafonica);
  Serial.write(',');
  Serial.write(';');

  // dados configuração faisca
  Serial.write('j');
  Serial.write(',');
  sendSerialInt(referencia_leitura_ignicao);
  Serial.write(',');
  sendSerialInt(modo_ignicao);
  Serial.write(',');
  sendSerialInt(grau_avanco_partida);
  Serial.write(',');
  sendSerialInt(avanco_fixo);
  Serial.write(',');
  sendSerialInt(grau_avanco_fixo);
  Serial.write(',');
  sendSerialInt(tipo_sinal_bobina);
  Serial.write(',');
  Serial.write(';');

  // dados configuração dwell
  Serial.write('k');
  Serial.write(',');
  sendSerialInt(dwell_partida);
  Serial.write(',');
  sendSerialInt(dwell_funcionamento);
  Serial.write(',');
  Serial.write(';');

  // dados configuração calibrate temperature sensor ctl
  Serial.write('l');
  Serial.write(','); // letra L minúsculo
  sendSerialInt(referencia_temperatura_clt1);
  Serial.write(',');
  sendSerialInt(referencia_resistencia_clt1);
  Serial.write(',');
  sendSerialInt(referencia_temperatura_clt2);
  Serial.write(',');
  sendSerialInt(referencia_resistencia_clt2);
  Serial.write(',');
  Serial.write(';');

  //---------ve-------------//
  Serial.write('d');
  Serial.write(',');
  // vetor map ou tps da ve
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_map_tps_ve[i]);
    if (i < 16) {
      Serial.write(',');
    }
  }
  Serial.write(';');
  
  Serial.write('e');
  Serial.write(',');
  // vetor rpm da tabela ve
  for (int i = 0; i < 16; i++) {
    sendSerialInt(vetor_rpm_ve[i]);
    if (i < 16) {
      Serial.write(',');
    }
  }
  Serial.write(';');
  
  // transforma matriz em vetor
  Serial.write('f');
  Serial.write(',');
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      sendSerialInt(matriz_ve[i][j]);
      Serial.write(',');
    }
  }
  Serial.write(';');
  //---------ve------------//  
  
  // dados configuração injeção 
  Serial.write('m');
  Serial.write(',');
  sendSerialInt(referencia_leitura_injecao);
  Serial.write(',');
  sendSerialInt(tipo_motor);
  Serial.write(',');
  sendSerialInt(modo_injecao);
  Serial.write(',');
  sendSerialInt(emparelhar_injetor);
  Serial.write(',');
  sendSerialInt(deslocamento_motor);
  Serial.write(',');
  sendSerialInt(numero_cilindro_injecao);
  Serial.write(',');
  sendSerialInt(numero_injetor);
  Serial.write(',');
  sendSerialInt(numero_esguicho);
  Serial.write(',');
  sendSerialInt(tamanho_injetor);
  Serial.write(',');
  sendSerialInt(tipo_acionamento_injetor);
  Serial.write(',');
  sendSerialInt(tipo_combustivel);
  Serial.write(',');
  sendSerialInt(REQ_FUEL);
  Serial.write(',');
  sendSerialInt(dreq_fuel);
  Serial.write(',');
  Serial.write(';');

  // dados configuração proteção e limites 
  Serial.write('n');
  Serial.write(',');
  sendSerialInt(tipo_protecao);
  Serial.write(',');
  sendSerialInt(rpm_pre_corte);
  Serial.write(',');
  sendSerialInt(avanco_corte);
  Serial.write(',');
  sendSerialInt(tempo_corte);
  Serial.write(',');
  sendSerialInt(rpm_maximo_corte);
  Serial.write(',');
  sendSerialInt(numero_base_corte);
  Serial.write(',');
  sendSerialInt(qtd_corte);
  Serial.write(',');
  Serial.write(';');

    // Dados de configuração de enriquecimento na aceleração
    Serial.write('o');
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[0]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[1]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[2]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[3]);
    Serial.write(',');
    sendSerialInt(enriquecimento_aceleracao[4]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[0]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[1]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[2]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[3]);
    Serial.write(',');
    sendSerialInt(tps_dot_escala[4]);
    Serial.write(',');
    sendSerialInt(tipo_verificacao_aceleracao_rapida);
    Serial.write(',');
    sendSerialInt(tps_mudanca_minima);
    Serial.write(',');
    sendSerialInt(intervalo_tempo_aceleracao);
    Serial.write(',');
    sendSerialInt(duracao_enriquecimento);
    Serial.write(',');
    sendSerialInt(rpm_minimo_enriquecimento);
    Serial.write(',');
    sendSerialInt(rpm_maximo_enriquecimento);
    Serial.write(',');
    sendSerialInt(enriquecimento_desaceleracao);
    Serial.write(',');
    Serial.write(';');

  // dados configuração TPS 
  Serial.write('p');
  Serial.write(',');
  sendSerialInt(valor_tps_minimo);
  Serial.write(',');
  sendSerialInt(valor_tps_maximo);
  Serial.write(',');
  Serial.write(';');

}


// void ler_dados_memoria(){
//       Serial.print(";a,");
//       // vetor map ou tps
//       for (int  i = 0; i < 16; i++){
//         Serial.print(vetor_map_tps[i]);
//         Serial.print(",");
//       }
//       Serial.print(";");
      
//       Serial.print("b,");
//       // vetor rpm
//       for (int  i = 0; i < 16; i++){
//         Serial.print(vetor_rpm[i]);
//         Serial.print(",");
//       }
//       Serial.print(";");
//       // transforma matriz em vetor
//       Serial.print("c,");
//         int k = 0;
//         for (int i = 0; i < 16; i++){
//           for (int j = 0; j < 16; j++){
//             Serial.print(matriz_avanco[i][j]);
//             Serial.print(",");
//             k++;
//           }
//         }
//         Serial.print(";");
      
//       // dados configuração inicial
//       Serial.print("g,");
//       Serial.print(tipo_ignicao);
//       Serial.print(",");
//       Serial.print(qtd_dente);
//       Serial.print(",");
//       Serial.print(local_rodafonica);
//       Serial.print(",");
//       Serial.print(qtd_dente_faltante);
//       Serial.print(",");
//       Serial.print(grau_pms);
//       Serial.print(",");
//       Serial.print(qtd_cilindro * local_rodafonica);
//       Serial.print(",");
//       Serial.print(";");

//       // dados configuração faisca
//       Serial.print("j,");
//       Serial.print(referencia_leitura_ignicao);
//       Serial.print(",");
//       Serial.print(modo_ignicao);
//       Serial.print(",");
//       Serial.print(grau_avanco_partida);
//       Serial.print(",");
//       Serial.print(avanco_fixo);
//       Serial.print(",");
//       Serial.print(grau_avanco_fixo);
//       Serial.print(",");
//       Serial.print(tipo_sinal_bobina);
//       Serial.print(",");
//       Serial.print(";");

//       // dados configuração dwell
//       Serial.print("k,");
//       Serial.print(dwell_partida);
//       Serial.print(",");
//       Serial.print(dwell_funcionamento);
//       Serial.print(",");
//       Serial.print(";");

//       // dados configuração calibrate temperature sensor ctl
//       Serial.print("l,");//letra L minusculo
//       Serial.print(referencia_temperatura_clt1);
//       Serial.print(",");
//       Serial.print(referencia_resistencia_clt1);
//       Serial.print(",");
//       Serial.print(referencia_temperatura_clt2);
//       Serial.print(",");
//       Serial.print(referencia_resistencia_clt2);
//       Serial.print(",");
//       Serial.print(";");

//       //---------ve-------------//
//       Serial.print("d,");
//       // vetor map ou tps da ve
//       for (int  i = 0; i < 16; i++){
//         Serial.print(vetor_map_tps_ve[i]);
//         Serial.print(",");
//       }
//       Serial.print(";");
//       Serial.print("e,");
//       // vetor rpm da tabela ve
//       for (int  i = 0; i < 16; i++){
//         Serial.print(vetor_rpm_ve[i]);
//         Serial.print(",");
//       }
//       Serial.print(";");
//       // transforma matriz em vetor
//       Serial.print("f,");
//         k = 0;
//         for (int i = 0; i < 16; i++){
//           for (int j = 0; j < 16; j++){
//             Serial.print(matriz_ve[i][j]);
//             Serial.print(",");
//             k++;
//           }
//         }
//         Serial.print(";");
//       //---------ve------------//  
//       // dados configuração injeção 
// Serial.print("m,");
// Serial.print(referencia_leitura_injecao);
// Serial.print(",");
// Serial.print(tipo_motor);
// Serial.print(",");
// Serial.print(modo_injecao);
// Serial.print(",");
// Serial.print(emparelhar_injetor);
// Serial.print(",");
// Serial.print(deslocamento_motor);
// Serial.print(",");
// Serial.print(numero_cilindro_injecao);
// Serial.print(",");
// Serial.print(numero_injetor);
// Serial.print(",");
// Serial.print(numero_esguicho);
// Serial.print(",");
// Serial.print(tamanho_injetor);
// Serial.print(",");
// Serial.print(tipo_acionamento_injetor);
// Serial.print(",");
// Serial.print(tipo_combustivel);
// Serial.print(",");
// Serial.print(REQ_FUEL);
// Serial.print(",");
// Serial.print(dreq_fuel);
// Serial.print(",");
// Serial.print(";");
// // dados configuração proteção e limites 
// Serial.print("n,");
// Serial.print(tipo_protecao);
// Serial.print(",");
// Serial.print(rpm_pre_corte);
// Serial.print(",");
// Serial.print(avanco_corte);
// Serial.print(",");
// Serial.print(tempo_corte);
// Serial.print(",");
// Serial.print(rpm_maximo_corte);
// Serial.print(",");
// Serial.print(numero_base_corte);
// Serial.print(",");
// Serial.print(qtd_corte);
// Serial.print(",");
// Serial.print(";");
// }