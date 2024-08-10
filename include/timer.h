void timerCallback();
void initializeTimerOne(unsigned long microseconds) {
  TCCR1A = 0; // clear control register A
  TCCR1B = _BV(WGM13); // set mode 8: phase and frequency correct pwm, stop the timer
  unsigned long cycles = (F_CPU / 2000000) * microseconds;
  unsigned char clockSelectBits;
  
  if (cycles < 65536) {
    clockSelectBits = _BV(CS10); // no prescale, full xtal
  } else if ((cycles >>= 3) < 65536) {
    clockSelectBits = _BV(CS11); // prescale by /8
  } else if ((cycles >>= 3) < 65536) {
    clockSelectBits = _BV(CS11) | _BV(CS10); // prescale by /64
  } else if ((cycles >>= 2) < 65536) {
    clockSelectBits = _BV(CS12); // prescale by /256
  } else if ((cycles >>= 2) < 65536) {
    clockSelectBits = _BV(CS12) | _BV(CS10); // prescale by /1024
  } else {
    cycles = 65535;
    clockSelectBits = _BV(CS12) | _BV(CS10); // request was out of bounds, set as maximum
  }

  // unsigned char oldSREG = SREG;
  // cli();
  // ICR1 = cycles;
  // SREG = oldSREG;
  // TCCR1B = _BV(WGM13) | clockSelectBits;
  // TIMSK1 = _BV(TOIE1); // sets the timer overflow interrupt enable bit
  // Desativa apenas a interrupção do Timer 1
  TIMSK1 &= ~_BV(TOIE1); // clear the timer overflow interrupt enable bit
  ICR1 = cycles; // Set the top value for the timer
  TCCR1B = _BV(WGM13) | clockSelectBits;
  // Reativa a interrupção do Timer 1
  TIMSK1 |= _BV(TOIE1); // sets the timer overflow interrupt enable bit
}

ISR(TIMER1_OVF_vect) {
  timerCallback();
}

void timerCallback() {
tempo_atual = micros();
// tempo_inicial_codigo = micros(); // Registra o tempo inicial      
if(local_rodafonica == 1 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
  
if(loop_timer >= qtd_cilindro){
  loop_timer = 0;
}
int i = loop_timer; //provisorio para teste 
  if (grau_pms <= 120) {
    if (grau_pms < 60 || rpm > 3000) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 
  }else{
    ajuste_pms = 0;
  }

  if ( i < qtd_cilindro/2){
    //colocar este dentro do if abaixo
    //tempo_final_codigo = micros(); // Registra o tempo final
    //tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false)){
    tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        ((tempo_atual - tempo_atual_proxima_ignicao[i]) + dwell_bobina >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 100){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        // digitalWrite(ignicao_pins[i], 1);
        setPinHigh(ignicao_pins[i]);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        ((tempo_atual - tempo_atual_proxima_injecao[i]) >= tempo_proxima_injecao[i] - (grau_fechamento_injetor * tempo_cada_grau)) && 
        revolucoes_sincronizada >= 1 && status_corte == 0){
        if(tipo_acionamento_injetor == 1){
          captura_req_fuel[i] = true;
          for (int j = 0; j < numero_injetor; j++){
          // digitalWrite(injecao_pins[j], 1);
          setPinHigh(injecao_pins[i]);
          }
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
        // digitalWrite(injecao_pins[i], 1);
        setPinHigh(injecao_pins[i]);
        tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
        inj_acionado[i] = true;
        inj_acionado[i+1] = false;
        captura_req_fuel[i+1] = false;
        }
      // VE = matriz_ve[procura_indice(valor_referencia_busca_tempo_injecao, vetor_map_tps_ve, 16)][procura_indice(rpm, vetor_rpm_ve, 16)];
    
    }
  }
  // for (int i = qtd_cilindro / 2; i < qtd_cilindro; i++){//original
    if (i >= qtd_cilindro / 2){  
    // tempo_atual = micros() ;
    //tempo_final_codigo = micros(); // Registra o tempo final  
    //tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false)){
      tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        ((tempo_atual - tempo_atual_proxima_ignicao[i]) + dwell_bobina >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 100){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        // digitalWrite(ignicao_pins[i - qtd_cilindro/2], 1);
        setPinHigh(ignicao_pins[i - qtd_cilindro/2]);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        ((tempo_atual - tempo_atual_proxima_injecao[i]) >= tempo_proxima_injecao[i] - (grau_fechamento_injetor * tempo_cada_grau)) && 
        revolucoes_sincronizada >= 1 && status_corte == 0){
          captura_req_fuel[i] = true;
        if(tipo_acionamento_injetor == 1){
          for (int j = 0; j < numero_injetor; j++){
          // digitalWrite(injecao_pins[j], 1);
          setPinHigh(injecao_pins[j]);
          }
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
        // digitalWrite(injecao_pins[i - qtd_cilindro/2], 1);
        setPinHigh(injecao_pins[i - qtd_cilindro/2]);
        tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
        inj_acionado[i] = true;
        inj_acionado[i+1] = false;
        captura_req_fuel[i+1] = false;
        }
      
    }
  }
     
    // for (int i = 0; i < qtd_cilindro; i++) {
    if ((captura_dwell[i] == true) && (ign_acionado[i] == true)){
        if ((tempo_atual - tempo_percorrido[i]) >= dwell_bobina ) {
            //verificar o tempo gasto nesta tarefa abaixo
            // verifica_posicao_sensor = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
            // if(posicao_atual_sensor >= verifica_posicao_sensor){
            //   //enviar_byte_serial(posicao_atual_sensor, 1);
            //   //enviar_byte_serial(verifica_posicao_sensor, 1);     
            //   captura_dwell[i] = false;
            //   if (i < qtd_cilindro/2) {
            //     digitalWrite(ignicao_pins[i], 0);
            //   }else{
            //     digitalWrite(ignicao_pins[i - qtd_cilindro/2], 0);
            //   }
            // }else{
              captura_dwell[i] = false;
              if (i < qtd_cilindro/2) {
                // digitalWrite(ignicao_pins[i], 0);
                setPinLow(ignicao_pins[i]);
              }else{
                // digitalWrite(ignicao_pins[i - qtd_cilindro/2], 0);
                setPinLow(ignicao_pins[i - qtd_cilindro/2]);
              }
            // }
            //enviar_byte_serial(0, 1);        
        }
    }
    // if (captura_req_fuel[i] == true) {
    //     tempo_atual = tempo_atual ;
    //     if ((tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao) {
    //       if(tipo_acionamento_injetor == 1){
    //         for (int j = 0; j < numero_injetor; j++){
    //           digitalWrite(injecao_pins[j], 0);
    //         }
    //       }
    //           captura_req_fuel[i] = false;
    //           if (i < qtd_cilindro/2) {
    //             digitalWrite(injecao_pins[i], 0);
    //           }else{
    //             digitalWrite(injecao_pins[i - qtd_cilindro/2], 0);
    //           }
          
    //     }
    // }
          if (captura_req_fuel[i] == true && inj_acionado[i] == true){
        if ((tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao ) {
          if(tipo_acionamento_injetor == 1){
              captura_req_fuel[i] = false;
            for (int j = 0; j < numero_injetor; j++){
              // digitalWrite(injecao_pins[j], LOW);
              setPinLow(injecao_pins[i]);
            }
          }
              if (i < qtd_cilindro/2) {
                // digitalWrite(injecao_pins[i], LOW);
                setPinLow(injecao_pins[i]);
                captura_req_fuel[i] = false;
              }
              if (i >= qtd_cilindro/2){
                // digitalWrite(injecao_pins[i - qtd_cilindro/2], LOW);
                setPinLow(injecao_pins[i - qtd_cilindro/2]);
                captura_req_fuel[i] = false;
              }      
      }
          }
    // }
  loop_timer++;
  }

if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0 ){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0

if(loop_timer >= qtd_cilindro){
  loop_timer = 0;
}
int i = loop_timer; //provisorio para teste 
  if (grau_pms <= 120) {
    if (grau_pms < 60 || rpm > 3000) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 
  }else{
    ajuste_pms = 0;
  }

    if((captura_dwell[i] == false) && (ign_acionado[i] == false)){
      tempo_proxima_ignicao[i] = ( ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i) ) * tempo_cada_grau;
    } 
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
        // tempo_proxima_ignicao[i+1] = ( ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i+1) ) * tempo_cada_grau;      
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
          // digitalWrite(injecao_pins[j], 1);
          setPinHigh(injecao_pins[j]);
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
          // digitalWrite(injecao_pins[i], 1);
          setPinHigh(injecao_pins[i]);
          tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
          inj_acionado[i] = true;
          inj_acionado[i+1] = false;
          captura_req_fuel[i+1] = false;
        } 
        // tempo_proxima_injecao[i+1] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i+1)) * tempo_cada_grau; 
        
  }

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
        if ((micros() - tempo_percorrido[i]) >= dwell_bobina) {
            captura_dwell[i] = false;
            // ign_acionado[i] = false;
            // digitalWrite(ignicao_pins[i], 0);
            setPinLow(ignicao_pins[i]);
            //delay(5); //um pequeno atraso
        }
    
  }  
      // if (captura_req_fuel[i] == true && inj_acionado[i] == true){
        if ((micros() - tempo_percorrido_inj[i]) >= tempo_injecao) {
          captura_req_fuel[i] = false;
          if (tipo_acionamento_injetor == 1){
            for (int j = 0; j < numero_injetor; j++){
              // digitalWrite(injecao_pins[j], LOW);
              setPinLow(injecao_pins[j]);
            }
          }
          // digitalWrite(injecao_pins[i], LOW);
          setPinLow(injecao_pins[i]);     
        }
      // }
loop_timer++;         
}
  // tempo_final_codigo = micros(); // Registra o tempo final  
  // tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;  
}
