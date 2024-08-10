
// Define funções auxiliares
inline int calcularAjustePMS(int grau_pms, int rpm) {
    return (grau_pms <= 120 && (grau_pms < 60 || rpm > 3000)) ? 180 : 0;
}

void configurarIgnicao(int i, int ajuste_pms) {
    tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
}

void iniciarDwell(int i) {
    captura_dwell[i] = true;
    tempo_percorrido[i] = tempo_atual;
    digitalWrite(ignicao_pins[i], HIGH);
    tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i];
    ign_acionado[i] = true;
    ign_acionado[(i + 1) % qtd_cilindro] = false;
    captura_dwell[(i + 1) % qtd_cilindro] = false;
}



void pararIgnicao(int i) {
    captura_dwell[i] = false;
    digitalWrite(ignicao_pins[i], LOW);
}

void processarCiclo() {
    if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0 ){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
        if (loop_timer >= qtd_cilindro) {
            loop_timer = 0;
        }
        int i = loop_timer;

        int ajuste_pms = calcularAjustePMS(grau_pms, rpm);

        if ((captura_dwell[i] == false) && (ign_acionado[i] == false)) {
            configurarIgnicao(i, ajuste_pms);
        }

        if ((captura_dwell[i] == false) && (ign_acionado[i] == false) &&
            (tempo_atual - tempo_atual_proxima_ignicao[i] + dwell_bobina >= tempo_proxima_ignicao[i]) &&
            revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 100) {
            iniciarDwell(i);
        }

        if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)) {
            configurarInjecao(i, ajuste_pms);
        }

        if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) &&
            (tempo_atual - tempo_atual_proxima_injecao[i] >= tempo_proxima_injecao[i] - (grau_fechamento_injetor * tempo_cada_grau)) &&
            revolucoes_sincronizada >= 1 && status_corte == 0) {
            iniciarInjecao(i);
        }

        if ((captura_dwell[i] == true) && (ign_acionado[i] == true) &&
            (tempo_atual - tempo_percorrido[i]) >= dwell_bobina) {
            pararIgnicao(i);
        }

        if (captura_req_fuel[i] == true && inj_acionado[i] == true &&
            (tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao) {
            pararInjecao(i);
        }

        loop_timer++;
    }
}

if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0 ){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0


  if (grau_pms <= 120) {
    if (grau_pms < 60 || rpm > 3000) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 
  }else{
    ajuste_pms = 0;
  }

   for (int i = 0; i < qtd_cilindro; i++){
    if((captura_dwell[i] == false) && (ign_acionado[i] == false)){
    tempo_proxima_ignicao[i] = ( ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i) ) * tempo_cada_grau;

    }
    // tempo_atual = tempo_atual;
    //tempo_final_codigo = tempo_atual; // Registra o tempo final  
    //tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000ul) >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 100){ 
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        // setPinHigh(ignicao_pins[i]);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;      
    }
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
    tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
    // tempo_atual = tempo_atual;
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_injecao[i] >= tempo_proxima_injecao[i] - (grau_fechamento_injetor * tempo_cada_grau)) && 
        revolucoes_sincronizada >= 1 && status_corte == 0){
        if(tipo_acionamento_injetor == 1){
          for (int j = 0; j < numero_injetor; j++){
          digitalWrite(injecao_pins[j], 1);
          // setPinHigh(injecao_pins[j]);
          }
          captura_req_fuel[i] = true;
          //tempo_final_codigo = tempo_atual; // Registra o tempo final  
          //tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
          // tempo_percorrido_inj[i] = tempo_atual - tempo_decorrido_codigo;
          tempo_percorrido_inj[i] = tempo_atual;
          tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
          inj_acionado[i] = true;
          inj_acionado[i+1] = false;
          captura_req_fuel[i+1] = false;
        }else{
          captura_req_fuel[i] = true;
          //tempo_final_codigo = tempo_atual; // Registra o tempo final  
          //tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
          // tempo_percorrido_inj[i] = tempo_atual - tempo_decorrido_codigo;
          tempo_percorrido_inj[i] = tempo_atual;
          digitalWrite(injecao_pins[i], 1);
          // setPinHigh(injecao_pins[i]);
          tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
          inj_acionado[i] = true;
          inj_acionado[i+1] = false;
          captura_req_fuel[i+1] = false;
        }  
        
    }
  }
    for (int i = 0; i < qtd_cilindro; i++){
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
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000ul)) {
            captura_dwell[i] = false;
            //ign_acionado[i] = false;
            digitalWrite(ignicao_pins[i], 0);
            // setPinLow(ignicao_pins[i]);
            //delay(5); //um pequeno atraso
        }
    
  }  
      if (captura_req_fuel[i] == true && inj_acionado[i] == true){
        if ((tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao) {
          captura_req_fuel[i] = false;
          if (tipo_acionamento_injetor == 1){
            for (int j = 0; j < numero_injetor; j++){
              digitalWrite(injecao_pins[j], LOW);
              // setPinLow(injecao_pins[i]);
            }
          }
          digitalWrite(injecao_pins[i], LOW);
          // setPinLow(injecao_pins[i]);     
        }
      }  
    }
             
}
