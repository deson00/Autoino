void calcula_grau_ignicao(int i){
if((captura_dwell[i] == false) && (ign_acionado[i] == false)){
      tempo_proxima_ignicao[i] = ( ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i) ) * tempo_cada_grau;
    } 
}
void iniciar_dwell(int i){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + dwell_bobina >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 100){ 
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        // digitalWrite(ignicao_pins[i], 1);
        setPinHigh(ignicao_pins[i]);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
        // tempo_proxima_ignicao[i+1] = (grau_pms - grau_avanco + (grau_entre_cada_cilindro * i+1) ) * tempo_cada_grau;      
    }
}

void desligar_dwell(int i){
  //tempo_atual = tempo_atual;
      if ((captura_dwell[i] == true) && (ign_acionado[i] == true)) {
            // verifica_posicao_sensor = ajuste_pms + grau_pms + grau_avanco + (grau_entre_cada_cilindro * i);
            // if(posicao_atual_sensor >= verifica_posicao_sensor){
            //   captura_dwell[i] = false;
            //   //ign_acionado[i] = false;
            //   digitalWrite(ignicao_pins[i], 0);
            //   //enviar_byte_serial(verifica_posicao_sensor, 1);
            //   //delay(5); //um pequeno atraso
            // }
       //tempo_atual = tempo_atual;     
        if ((tempo_atual - tempo_percorrido[i]) >= dwell_bobina) {
            captura_dwell[i] = false;
            // ign_acionado[i] = false;
            // digitalWrite(ignicao_pins[i], 0);
            setPinLow(ignicao_pins[i]);
            //delay(5); //um pequeno atraso
        }
    
  }  
}