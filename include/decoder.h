void leitor_sensor_roda_fonica()
{
  noInterrupts();
  // tempo_inicial_codigo = micros(); // Registra o tempo inicial
  qtd_leitura++;
  tempo_atual = micros() ;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);
  //verifica_falha = (tempo_dente_anterior[leitura] / 2) + tempo_dente_anterior[leitura];
  // verifica_falha = (tempo_dente_anterior[leitura] / 2) + (tempo_dente_anterior[leitura] * qtd_dente_faltante);
  verifica_falha = (tempo_dente_anterior[leitura] >> 1) + (tempo_dente_anterior[leitura] * qtd_dente_faltante);

  if (inicia_tempo_sensor_roda_fonica){
    tempo_anterior = tempo_atual;
    tempo_dente_anterior[0] = tempo_anterior;
    inicia_tempo_sensor_roda_fonica = 0;
  }
  if (leitura == 0){
    leitura = 1;
    tempo_dente_anterior[0] = intervalo_tempo_entre_dente;
  }
  else{
    leitura = 0;
    tempo_dente_anterior[1] = intervalo_tempo_entre_dente;
  }
  //Serial.print("|");
  //Serial.print(qtd_leitura);
  // if (verifica_falha < intervalo_tempo_entre_dente && (intervalo_tempo_entre_dente < (tempo_dente_anterior[leitura] * (qtd_dente_faltante * 4)))) //linha original 
if (verifica_falha < intervalo_tempo_entre_dente && (intervalo_tempo_entre_dente < (tempo_dente_anterior[leitura] * (qtd_dente_faltante << 4))))
  {
    if (qtd_voltas == 1){
      tempo_final_volta_completa = tempo_atual;
      tempo_total_volta_completa = (tempo_final_volta_completa - tempo_inicio_volta_completa);
      qtd_voltas = 0;
    }
    if (qtd_voltas == 0){
      tempo_inicio_volta_completa = tempo_atual;
      qtd_voltas = 1;
    }
    //Serial.print("__");
    //Serial.println("");
    //Serial.print(posicao_atual_sensor); 
    qtd_revolucoes++;
    // tempo_cada_grau = tempo_total_volta_completa / 360;
    tempo_cada_grau = tempo_total_volta_completa * 0.00277777778; //0.00277777778 alternativa para melhorar a velocidade do codigo 

    // posicao_atual_sensor = grau_cada_dente * qtd_dente_faltante;
    posicao_atual_sensor = 0;
   if ((qtd_leitura != (qtd_dente - qtd_dente_faltante))) {
      if(++qtd_perda_sincronia >=255){
        qtd_perda_sincronia = 0;
      }
    }else{
      revolucoes_sincronizada++;
    }
    qtd_leitura = 0; 
    if(tipo_ignicao_sequencial == 0 ){    
    tempo_atual_proxima_ignicao[0] = tempo_atual;
    ign_acionado[0] = false;
    captura_dwell[0] = false; 
    tempo_atual_proxima_injecao[0] = tempo_atual;
    inj_acionado[0] = false;
    captura_req_fuel[0] = false; 
    }
  }else{
    if(rpm < rpm_partida){
    tempo_cada_grau = intervalo_tempo_entre_dente / (360 / qtd_dente);
    }
    //enviar_byte_serial(tempo_cada_grau / 1000, 1);
  }
  // posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;
  tempo_anterior = tempo_atual;
  // tempo_final_codigo = micros(); // Registra o tempo final
  // tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
  interrupts();
}