const unsigned long MARGEM_MINIMA_DWELL_US = 500UL;

static inline bool referencia_angular_valida_ignicao(int i){
    if (rpm >= rpm_partida) {
        return true;
    }

    long posicao_grau_atual = (long)posicao_atual_sensor * (long)grau_cada_dente;
    long limiar_referencia = (long)ajuste_pms + (long)grau_pms - (long)grau_cada_dente + ((long)grau_entre_cada_cilindro * (long)i);
    return posicao_grau_atual >= limiar_referencia;
}

static inline unsigned long calcula_tempo_proxima_ignicao_seguro(int i) {
    unsigned long tempo_alvo = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    unsigned long tempo_minimo_para_dwell = dwell_bobina + MARGEM_MINIMA_DWELL_US;
    unsigned long tempo_decorrido = tempo_atual - tempo_atual_proxima_ignicao[i];

    if ((tempo_decorrido + tempo_minimo_para_dwell) > tempo_alvo) {
        unsigned long deslocamento_seguranca_graus = 360UL;
        if (local_rodafonica == 2 && tipo_ignicao_sequencial == 0) {
            deslocamento_seguranca_graus = 180UL;
        }
        tempo_alvo += deslocamento_seguranca_graus * tempo_cada_grau;
    }
    return tempo_alvo;
}

void calcula_grau_ignicao(int i){
if((captura_dwell[i] == false) && (ign_acionado[i] == false) && referencia_angular_valida_ignicao(i)){
      tempo_proxima_ignicao[i] = calcula_tempo_proxima_ignicao_seguro(i);
    } 
}
void iniciar_dwell(int i){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
                referencia_angular_valida_ignicao(i) &&
        (tempo_atual - tempo_atual_proxima_ignicao[i] + dwell_bobina >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 100){ 
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        // setPinHigh(ignicao_pins[i]);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
        // tempo_proxima_ignicao[i+1] = (grau_pms - grau_avanco + (grau_entre_cada_cilindro * i+1) ) * tempo_cada_grau;      
    }
}

void iniciar_dwell_comandoA(int i){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        ((tempo_atual - tempo_atual_proxima_ignicao[i]) + dwell_bobina >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 10){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        // setPinHigh(ignicao_pins[i]);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
}

void iniciar_dwell_comandoB(int i){
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        ((tempo_atual - tempo_atual_proxima_ignicao[i]) + dwell_bobina >= tempo_proxima_ignicao[i]) && 
        revolucoes_sincronizada >= 1 && status_corte == 0 && rpm > 10){
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
            digitalWrite(ignicao_pins[i], 0);
            // setPinLow(ignicao_pins[i]);
            //delay(5); //um pequeno atraso
        }
    
  }  
}
