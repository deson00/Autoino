
void enviar_byte_serial(int valor, int tamanho) {
  if (tamanho == 1) {
    // Verifica se o valor é um caractere
    if (valor >= 0 && valor <= 255) {
      // Se for um caractere, envia o byte diretamente
      Serial.write((char)valor);
    } else {
      // Se não for um caractere, envia apenas o byte menos significativo
      char byteBaixo = valor & 0xFF;  // Os 8 bits menos significativos
      Serial.write(byteBaixo);
    }
  } else if (tamanho == 2) {
    // Divide o valor em dois bytes
    char byteBaixo = valor & 0xFF;        // Os 8 bits menos significativos
    char byteAlto = (valor >> 8) & 0xFF;  // Os 8 bits mais significativos

    // Envia os dois bytes pela porta serial
    Serial.write(byteBaixo);
    Serial.write(byteAlto);
  }
}
void envia_dados_tempo_real(int indice_envio){
    if (status_dados_tempo_real){
      if(indice_envio == 1){
        //enviar_byte_serial(verifica_posicao_sensor, 2);
        //enviar_byte_serial(posicao_atual_sensor, 2);
        enviar_byte_serial(rpm_anterior, 2); 
        enviar_byte_serial(valor_map, 2);
        enviar_byte_serial(temperatura_motor, 1);
        enviar_byte_serial(grau_avanco, 1);
        enviar_byte_serial(qtd_loop*5, 2);
        enviar_byte_serial(qtd_perda_sincronia, 1);
        enviar_byte_serial(VE, 1);
        enviar_byte_serial(tempo_injecao, 2);
        enviar_byte_serial(valor_tps, 1);
        enviar_byte_serial(status_corte, 1);
        enviar_byte_serial(tps_dot_porcentagem, 2);
      }
    } 
}