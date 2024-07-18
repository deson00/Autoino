

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
        enviar_byte_serial(tps_dot_porcentagem_aceleracao, 2);
        enviar_byte_serial(sonda_narrow, 2);
        enviar_byte_serial(valor_tps_adc, 2);
      }
    } 
}