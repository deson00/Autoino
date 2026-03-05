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

static inline byte indice_pino_injecao(int i) {
  if (local_rodafonica == 1 && i >= (qtd_cilindro / 2)) {
    return (byte)(i - (qtd_cilindro / 2));
  }
  return (byte)i;
}

void calcula_grau_injetor(int i){
if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      int angulo_base_injecao = ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i) - grau_fechamento_injetor;
      while (angulo_base_injecao < 0) {
        angulo_base_injecao += 360;
      }
      if (angulo_base_injecao >= 360) {
        angulo_base_injecao = angulo_base_injecao % 360;
      }
      if (angulo_base_injecao == 0) {
        angulo_base_injecao = 1;
      }
      tempo_proxima_injecao[i] = (unsigned long)angulo_base_injecao * tempo_cada_grau;
    }
}
void ligar_injetor(int i){
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        revolucoes_sincronizada >= 1 && status_corte == 0){
        if(tipo_acionamento_injetor == 1){
          for (int j = 0; j < numero_injetor; j++){
          digitalWrite(injecao_pins[j], HIGH);
          //setPinHigh(injecao_pins[j]);
          }
          captura_req_fuel[i] = true;
          tempo_percorrido_inj[i] = micros();
          inj_acionado[i] = true;
        }else{
          byte pino = indice_pino_injecao(i);
          captura_req_fuel[i] = true;
          tempo_percorrido_inj[i] = micros();
          digitalWrite(injecao_pins[pino], HIGH);
          //setPinHigh(injecao_pins[i]);
          inj_acionado[i] = true;
        } 
  }
}

void desligar_injetor(int i){
  if (captura_req_fuel[i] == true && inj_acionado[i] == true){
          captura_req_fuel[i] = false;
          inj_acionado[i] = false;
          if (tipo_acionamento_injetor == 1){
            for (int j = 0; j < numero_injetor; j++){
              digitalWrite(injecao_pins[j], LOW);
              // setPinLow(injecao_pins[j]);
            }
          } else {
            byte pino = indice_pino_injecao(i);
            digitalWrite(injecao_pins[pino], LOW);
            // setPinLow(injecao_pins[i]);
          }
  }
}

void calcula_grau_injetor_comando(int i){
    if ( i < qtd_cilindro/2){
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
  }
    if (i >= qtd_cilindro / 2){
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    }
  }
}

void ligar_injetor_comando(int i){
    ligar_injetor(i);
}

void desligar_injetor_comando(){
         for (int i = 0; i < qtd_cilindro; i++)
         {
          desligar_injetor(i);
    }
}