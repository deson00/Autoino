#define MIN_INTERVALO_DENTE_US 50        // Filtro anti-bounce mínimo
volatile uint32_t ultimo_tempo_interrupcao = 0;
void decoder_roda_fonica_padrao(){//roda fonica padrao com quantidade de dente - dente faltante
 // tempo_inicial_codigo = micros(); // Registra o tempo inicial
  uint32_t tempo_agora = micros();
  if ((tempo_agora - ultimo_tempo_interrupcao) < MIN_INTERVALO_DENTE_US) {
        return; // Ignora o pulso, é ruído. Não faz nada, pois não é um pulso válido.
    }
  ultimo_tempo_interrupcao = tempo_agora;  
  qtd_leitura++;
  tempo_atual = tempo_agora ;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);
  //verifica_falha = (tempo_dente_anterior[leitura] / 2) + tempo_dente_anterior[leitura];
  // verifica_falha = (tempo_dente_anterior[leitura] / 2) + (tempo_dente_anterior[leitura] * qtd_dente_faltante);
  // verifica_falha = (tempo_dente_anterior[leitura] >> 1) + (tempo_dente_anterior[leitura] * qtd_dente_faltante);
  verifica_falha = (tempo_dente_anterior[leitura] >> 1) + tempo_dente_anterior[leitura];
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
// if (verifica_falha < intervalo_tempo_entre_dente && (intervalo_tempo_entre_dente < (tempo_dente_anterior[leitura] * (qtd_dente_faltante << 4))))
if (verifica_falha < intervalo_tempo_entre_dente)
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
    if(tipo_ignicao_sequencial == 0){
        tempo_atual_proxima_ignicao[0] = tempo_atual;
        ign_acionado[0] = false;
        captura_dwell[0] = false; 
        tempo_atual_proxima_injecao[0] = tempo_atual;
        inj_acionado[0] = false;
        captura_req_fuel[0] = false;
     
    }

  }else{
    posicao_atual_sensor++;
    if(rpm < rpm_partida)
    {
      tempo_cada_grau = intervalo_tempo_entre_dente / (360 / qtd_dente);
      if(local_rodafonica == 1 && tipo_ignicao_sequencial == 0)
      {
        if (grau_pms > 180)
        {
          ajuste_pms = 180;
        }
        else{
          ajuste_pms = 0;
        }
        
        for (int i = 0; i < qtd_cilindro; i++)
        {
            if ( i < qtd_cilindro/2)
              {
                if (posicao_atual_sensor * grau_cada_dente >= grau_pms - ajuste_pms + (grau_entre_cada_cilindro * i) && (captura_dwell[i] == false) && (ign_acionado[i] == false)) 
                  {
                    captura_dwell[i] = true;
                    tempo_percorrido[i] = tempo_atual;
                    digitalWrite(ignicao_pins[i], 1);
                    // setPinHigh(ignicao_pins[i]);
                    tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
                    ign_acionado[i] = true;
                    ign_acionado[i+1] = false;
                    captura_dwell[i+1] = false;
                    // enviar_byte_serial(posicao_atual_sensor * grau_cada_dente, 1);
                  }
              }
          if (i >= qtd_cilindro / 2)
            {
              if (posicao_atual_sensor * grau_cada_dente >= grau_pms - ajuste_pms + (grau_entre_cada_cilindro * i) && (captura_dwell[i] == false) && (ign_acionado[i] == false)) 
                {
                  captura_dwell[i] = true;
                  tempo_percorrido[i] = tempo_atual;
                  digitalWrite(ignicao_pins[i - qtd_cilindro/2], 1);
                  // setPinHigh(ignicao_pins[i - qtd_cilindro/2]);
                  tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
                  ign_acionado[i] = true;
                  ign_acionado[i+1] = false;
                  captura_dwell[i+1] = false;
                }
            }
        }
      }
    }
    //enviar_byte_serial(tempo_cada_grau / 1000, 1);

  }
  // posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;
  tempo_anterior = tempo_atual;
  // tempo_final_codigo = micros(); // Registra o tempo final
  // tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
}
void leitor_sensor_roda_fonica()
{
//  tempo_inicial_codigo = micros(); // Registra o tempo inicial 
  // noInterrupts();
  //  TIMSK1 &= ~_BV(TOIE1);  // Desativa a interrupção do Timer1
 decoder_roda_fonica_padrao();
//  TIMSK1 |= _BV(TOIE1);   // Reativa a interrupção do Timer1
  // interrupts();
// tempo_final_codigo = micros(); // Registra o tempo final  
// tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;   
}