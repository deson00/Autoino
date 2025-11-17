// Função para calcular a largura do pulso de injeção
float tempo_pulso_ve(float REQ_FUEL, float VE) {
    const float tempo_pulso_ve = REQ_FUEL * (VE / 100.0);
    return tempo_pulso_ve * 1000ul;
}
// Função para calcular a largura do pulso enriquecimento
float tempo_enriquecimento_gama(float valor_referencia, float correcao_aquecimento, float correcao_O2, float correcao_temperatura_ar, float correcao_barometrica) {
    const float REQ_FUEL_CONSTANT = 100; 
    const float enriquecimento_gama = (correcao_aquecimento / REQ_FUEL_CONSTANT) * (correcao_O2 / REQ_FUEL_CONSTANT) *
                               (correcao_temperatura_ar / REQ_FUEL_CONSTANT) * (correcao_barometrica / REQ_FUEL_CONSTANT);
 
    return valor_referencia * enriquecimento_gama;
}
void calcula_grau_injetor(int i){
if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      tempo_proxima_injecao[i] = ((ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau) - (grau_fechamento_injetor * tempo_cada_grau);
    }
}
void ligar_injetor(int i){
    tempo_atual = micros();
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_injecao[i] >= tempo_proxima_injecao[i] ) && 
        revolucoes_sincronizada >= 1 && status_corte == 0){
        if(tipo_acionamento_injetor == 1){
          for (int j = 0; j < numero_injetor; j++){
          digitalWrite(injecao_pins[j], 1);
          //setPinHigh(injecao_pins[j]);
          }
          captura_req_fuel[i] = true;
          tempo_percorrido_inj[i] = tempo_atual;
          tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
          inj_acionado[i] = true;
          inj_acionado[i+1] = false;
          captura_req_fuel[i+1] = false;
        }else{
          captura_req_fuel[i] = true;
          tempo_percorrido_inj[i] = tempo_atual;
          digitalWrite(injecao_pins[i], 1);
          //setPinHigh(injecao_pins[i]);
          tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
          inj_acionado[i] = true;
          inj_acionado[i+1] = false;
          captura_req_fuel[i+1] = false;
        } 
        // tempo_proxima_injecao[i+1] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i+1)) * tempo_cada_grau; 
        
  }
}

void desligar_injetor(int i){
  tempo_atual = micros() ;
  // tempo_atual += 1200;
  if (captura_req_fuel[i] == true && inj_acionado[i] == true){
    if ((tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao) {
          captura_req_fuel[i] = false;
          if (tipo_acionamento_injetor == 1){
            for (int j = 0; j < numero_injetor; j++){
              digitalWrite(injecao_pins[j], LOW);
              // setPinLow(injecao_pins[j]);
            }
          }
          digitalWrite(injecao_pins[i], LOW);
          // setPinLow(injecao_pins[i]);     
    }    
  }
}