#include <Arduino.h>
#include <EEPROM.h>
#include <definicoes_hardware.h>
#include <variaveis_global.h>
#include <ler_dados_eeprom.h>
#include <ler_dados_memoria.h>
#include <grava_dados_eeprom.h>
#include <leitura_entrada_dados_serial.h>
#include <envia_dados_tempo_real.h>
#include <util.h>
#include <sensores.h>
#include <protecao.h>
#include <decoder.h>
// Função para calcular a largura do pulso de injeção
float calcularLarguraPulso(float REQ_FUEL, float MAP, float VE, float GammaE, float InjOpenTime) {
    // Constantes
    const float REQ_FUEL_CONSTANT = 100; // Supondo que REQ_FUEL está em percentagem

    // Calcular fator de enriquecimento (E)
    const float Warmup = 0; // Defina aqui o valor de enriquecimento do aquecimento
    const float O2_ClosedLoop = 0; // Defina aqui o valor de enriquecimento do O2 em Malha Fechada
    const float AirCorr = 0; // Defina aqui o valor de correção do ar
    const float BaroCorr = 0; // Defina aqui o valor de correção do barômetro

    const float gamma_Enrich = (Warmup / REQ_FUEL_CONSTANT) * (O2_ClosedLoop / REQ_FUEL_CONSTANT) *
                               (AirCorr / REQ_FUEL_CONSTANT) * (BaroCorr / REQ_FUEL_CONSTANT);
    // Calcular a largura do pulso (PW)
    const float PW = REQ_FUEL * MAP / REQ_FUEL_CONSTANT * VE / REQ_FUEL_CONSTANT * GammaE / REQ_FUEL_CONSTANT + InjOpenTime;
    return PW;
}

void setup(){
  ler_dados_eeprom();//aqui le os dados da eeprom que forem salvo anteriormente
  delay(1000);  
  pinMode(ign1, OUTPUT);
  pinMode(ign2, OUTPUT);
  pinMode(ign3, OUTPUT);
  pinMode(ign4, OUTPUT);
  pinMode(inj1, OUTPUT);
  pinMode(inj2, OUTPUT);
  pinMode(inj3, OUTPUT);
  pinMode(inj4, OUTPUT);
  pinMode(pino_sensor_roda_fonica, INPUT_PULLUP);
  pinMode(pino_sensor_map, INPUT);
  pinMode(pino_sensor_tps, INPUT);
  pinMode(pino_sensor_clt, INPUT);
  pinMode(pino_sensor_iat, INPUT);
  pinMode(pino_sensor_o2, INPUT);
  pinMode(pino_sensor_brv, INPUT);

  attachInterrupt(digitalPinToInterrupt(pino_sensor_roda_fonica), leitor_sensor_roda_fonica, RISING);
  Serial.begin(9600);
}
void loop(){ 
    qtd_loop++;
    //tempo_inicial_codigo = micros(); // Registra o tempo inicial
    //tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo;
    // if(contador_leitura >=10){
    //   // Ordena as leituras e encontra a mediana
    //   sort(leituras_map, contador_leitura);
    //   sort(leituras_tps, contador_leitura);
    //   // Mapeia os valores médios para o intervalo desejado
    //   valor_map = map(leituras_map[contador_leitura / 2], 0, 1023, valor_map_minimo, valor_map_maximo);
    //   valor_tps = map(leituras_tps[contador_leitura / 2], 0, 1023, valor_tps_minimo, valor_tps_maximo);
    //   contador_leitura = 0;
    //   tempo_final_codigo = micros(); // Registra o tempo final
    // }
    
    //leituras_map[contador_leitura++] = analogRead(pino_sensor_map);
    //leituras_tps[contador_leitura++] = analogRead(pino_sensor_tps);
    valor_map = map(analogRead(pino_sensor_map), 0, 1023, valor_map_minimo, valor_map_maximo);
    valor_tps = map(analogRead(pino_sensor_tps), 0, 1023, valor_tps_minimo, valor_tps_maximo);
    if(referencia_leitura_ignicao == 1){
      valor_referencia_busca_avanco = valor_map;   
    }else{
      valor_referencia_busca_avanco = valor_tps;
    }
    if(referencia_leitura_injecao == 1){
      valor_referencia_busca_tempo_injecao = valor_map;   
    }else{
      valor_referencia_busca_tempo_injecao = valor_tps;
    }
    VE = matriz_ve[procura_indice(valor_referencia_busca_avanco, vetor_map_tps_ve, 16)][procura_indice(rpm, vetor_rpm_ve, 16)];
    // Chama a função para calcular a largura do pulso
    tempo_injecao = calcularLarguraPulso(REQ_FUEL/1000, valor_map, VE, GammaE, InjOpenTime) * 1000ul;
      
    if(rpm < rpm_partida){
      grau_avanco = grau_avanco_partida;
      dwell_bobina = dwell_partida;
    }
    else if(avanco_fixo){
      grau_avanco = grau_avanco_fixo;
      dwell_bobina = dwell_funcionamento;
    }
    else if(rpm < 10000 && busca_avanco_linear == true){
      int grau_minimo = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)];
      int indice_rpm_minimo = procura_indice(rpm, vetor_rpm, 16);
      int grau_maximo = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)+1];
      int grau_linear = busca_linear(rpm, vetor_rpm[indice_rpm_minimo], grau_minimo, vetor_rpm[indice_rpm_minimo+1], grau_maximo);
      grau_avanco = grau_linear;
      dwell_bobina = dwell_funcionamento;
    }
    else{
      grau_avanco = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)];
      dwell_bobina = dwell_funcionamento;
    }
  
tempo_atual = micros() ;//salva sempre o tempo atual para verificaçoes
  
if(local_rodafonica == 1 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
  if (grau_pms <= 180) {
    if (grau_pms < 60 || rpm > 3000) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 
  }

  for (int i = 0; i < qtd_cilindro/2; i++){
    tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    tempo_atual = micros();
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000ul) >= tempo_proxima_ignicao[i]) && 
        (revolucoes_sincronizada >= 1)){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
    tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_injecao[i] >= tempo_proxima_injecao[i] - tempo_injecao - (grau_fechamento_injetor * tempo_cada_grau)) && 
        (revolucoes_sincronizada >= 1)){
        captura_req_fuel[i] = true;
        tempo_percorrido_inj[i] = tempo_atual;
        digitalWrite(injecao_pins[i], 1);
        tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
        inj_acionado[i] = true;
        inj_acionado[i+1] = false;
        captura_req_fuel[i+1] = false;
    }
  }
  for (int i = qtd_cilindro / 2; i < qtd_cilindro; i++){  
    tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    tempo_atual = micros() ;
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000ul) >= tempo_proxima_ignicao[i]) && 
        (revolucoes_sincronizada >= 1)){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i - qtd_cilindro/2], 1);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
    tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_injecao[i] >= tempo_proxima_injecao[i] - tempo_injecao - (grau_fechamento_injetor * tempo_cada_grau)) && 
        (revolucoes_sincronizada >= 1)){
        captura_req_fuel[i] = true;
        tempo_percorrido_inj[i] = tempo_atual;
        digitalWrite(injecao_pins[i - qtd_cilindro/2], 1);
        tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
        inj_acionado[i] = true;
        inj_acionado[i+1] = false;
        captura_req_fuel[i+1] = false;
    }
  }
  for (int i = 0; i < qtd_cilindro; i++) {
    tempo_atual = micros() ;
    if (captura_dwell[i] == true) {
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000ul)) {
            verifica_posicao_sensor = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
            if(posicao_atual_sensor >= verifica_posicao_sensor){     
              captura_dwell[i] = false;
              if (i < qtd_cilindro/2) {
                digitalWrite(ignicao_pins[i], 0);
              }else{
                digitalWrite(ignicao_pins[i - qtd_cilindro/2], 0);
              }
            }else{
              captura_dwell[i] = false;
              if (i < qtd_cilindro/2) {
                digitalWrite(ignicao_pins[i], 0);
              }else{
                digitalWrite(ignicao_pins[i - qtd_cilindro/2], 0);
              }
            }        
        }
    }
    if (captura_req_fuel[i] == true) {
        if ((tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao) {
              captura_req_fuel[i] = false;
              if (i < qtd_cilindro/2) {
                digitalWrite(injecao_pins[i], 0);
              }else{
                digitalWrite(injecao_pins[i - qtd_cilindro/2], 0);
              }       
        }
    }
  }
}

if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
  if (grau_pms <= 180) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 

   for (int i = 0; i < qtd_cilindro; i++){
    tempo_proxima_ignicao[i] = ( ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i) ) * tempo_cada_grau;
    tempo_atual = micros();
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000ul) >= tempo_proxima_ignicao[i]) && 
        (revolucoes_sincronizada >= 1)){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;        
    }
    tempo_proxima_injecao[i] = (ajuste_pms + grau_pms + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_injecao[i] >= tempo_proxima_injecao[i] - tempo_injecao - (grau_fechamento_injetor * tempo_cada_grau)) && 
        (revolucoes_sincronizada >= 1)){
        if(tipo_acionamento_injetor == 1){
          for (int j = 0; j < qtd_cilindro; j++){
          digitalWrite(injecao_pins[j], 1);
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
          tempo_atual_proxima_injecao[i + 1] = tempo_atual_proxima_injecao[i]; 
          inj_acionado[i] = true;
          inj_acionado[i+1] = false;
          captura_req_fuel[i+1] = false;
        }  
        
    }
  }
    for (int i = 0; i < qtd_cilindro; i++) {
      tempo_atual = micros();
      if ((captura_dwell[i] == true) && (ign_acionado[i] == true)) {
            verifica_posicao_sensor = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
            if(posicao_atual_sensor >= verifica_posicao_sensor){
              captura_dwell[i] = false;
              //ign_acionado[i] = false;
              digitalWrite(ignicao_pins[i], 0);
              //delay(5); //um pequeno atraso
            }
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000ul)) {
            captura_dwell[i] = false;
            //ign_acionado[i] = false;
            digitalWrite(ignicao_pins[i], 0);
            //delay(5); //um pequeno atraso
        }
    }
    if (captura_req_fuel[i] == true) {
        if ((tempo_atual - tempo_percorrido_inj[i]) >= tempo_injecao) {
          if(tipo_acionamento_injetor == 1){
            for (int j = 0; j < qtd_cilindro; j++){
              digitalWrite(injecao_pins[j], 0);
            }
          }
          captura_req_fuel[i] = false;
          digitalWrite(injecao_pins[i], 0);
                    
        }
    }
  }

}

  leitura_entrada_dados_serial(); 
  // verifica se já passou o intervalo de tempo
  if (millis() - ultima_execucao >= intervalo_execucao){     
  rpm_anterior = rpm; 
  envia_dados_tempo_real(1);
  temperatura_motor = temperatura_clt();
  protege_ignicao();
  //Serial.println(qtd_loop*(1000/intervalo_execucao)); 
  //Serial.println(freeMemory()); 
   // atualiza o tempo da última execução
   ultima_execucao = millis();
   qtd_loop = 0;
  }
}
