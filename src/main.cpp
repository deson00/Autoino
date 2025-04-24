#include <Arduino.h>
#include <EEPROM.h>
#include <definicoes_hardware.h>
#include <variaveis_global.h>
#include <util.h>
#include <ler_dados_eeprom.h>
#include <ler_dados_memoria.h>
#include <grava_dados_eeprom.h>
#include <leitura_entrada_dados_serial.h>
#include <envia_dados_tempo_real.h>
#include <sensores.h>
#include <protecao.h>
#include <decoder.h>
#include <injecao.h>
#include <ignicao.h>
#include <timer.h>
#include <timer2.h>
#include <enriquecimento_aceleracao.h>

// Função para calcular a RPM
// void calcularRPM() {
//   static unsigned long last_rpm_calculation_time = 0;
//   unsigned long current_time = millis();
//   unsigned long time_since_last_calculation = current_time - last_rpm_calculation_time;

//   // Calcule o RPM somente após uma volta completa ser detectada
//   if (qtd_voltas == 0 && tempo_total_volta_completa > 0 && time_since_last_calculation > 100) { // Adiciona um pequeno debounce
//       // tempo_total_volta_completa está em microssegundos
//       rpm = 60000000.0 / tempo_total_volta_completa; // Converte microssegundos por volta para RPM
//       tempo_total_volta_completa = 0; // Reseta para a próxima volta
//       last_rpm_calculation_time = current_time;
//   }
// }
void calcularRPM() {
  unsigned long revolucoes = qtd_revolucoes;  // Captura o valor atual de revoluções
  qtd_revolucoes = 0;  // Reseta o contador de revoluções
  unsigned long tempo_atual_local = micros();  // Captura o valor atual de tempo
  if (revolucoes > 0) {
    tempo_final_rpm = tempo_atual_local;
    volatile unsigned long delta = tempo_final_rpm - tempo_inicial_rpm;
    if(local_rodafonica == 1){
      rpm = (revolucoes * 60) / (float(delta) / 1000000) * 2;
    }else{
      rpm = (revolucoes * 60) / (float(delta) / 1000000);
    } 
    tempo_inicial_rpm = tempo_final_rpm;
  }
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
  // Inicializa o Timer 1 para gerar uma interrupção a cada 1 microsegundo
  initializeTimerOne(100);
  // initializeTimerTwo(200);
  
  sei(); // Habilita interrupções globais
  // Imprime uma mensagem dependendo do microcontrolador
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    Serial.println("Define OK: Arduino Uno ou Nano detectado.");
  #elif defined(__AVR_ATmega2560__)
    Serial.println("Define OK: Arduino Mega 2560 detectado.");
  #else
    Serial.println("Define não reconhecido: Microcontrolador desconhecido.");
  #endif
}
void loop(){
   calcularRPM();
    qtd_loop++;
    //tempo_inicial_codigo = micros(); // Registra o tempo inicial
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
    valor_tps_adc = analogRead(pino_sensor_tps);
    valor_tps = map(valor_tps_adc, valor_tps_minimo, valor_tps_maximo, 0, 100);
    valor_o2 = analogRead(pino_sensor_o2);
    sonda_narrow = valor_o2 * (1000.0 / 1023.0);
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
         
    if(rpm < rpm_partida){
      grau_avanco = grau_avanco_partida;
      dwell_bobina = dwell_partida * 1000ul; 
      status_corte = 0;
    }
    else if(avanco_fixo){
      grau_avanco = grau_avanco_fixo;
      dwell_bobina = dwell_funcionamento * 1000ul;
      status_corte = 0;
    }
    else if(tipo_protecao != 0 && rpm_anterior >= rpm_pre_corte){
      grau_avanco = avanco_corte;
      status_corte = 1;
      dwell_bobina = dwell_funcionamento * 1000ul;
    }
    else if(rpm < 7000 && busca_avanco_linear == true){
      int grau_minimo = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)];
      int indice_rpm_minimo = procura_indice(rpm, vetor_rpm, 16);
      int grau_maximo = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)+1];
      int grau_linear = busca_linear(rpm, vetor_rpm[indice_rpm_minimo], grau_minimo, vetor_rpm[indice_rpm_minimo+1], grau_maximo);
      grau_avanco = grau_linear;
      dwell_bobina = dwell_funcionamento * 1000ul;
      status_corte = 0;
    }
    else{
      grau_avanco = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)];
      dwell_bobina = dwell_funcionamento * 1000ul;
      status_corte = 0;
    }

VE = matriz_ve[procura_indice(valor_referencia_busca_tempo_injecao, vetor_map_tps_ve, 16)][procura_indice(rpm, vetor_rpm_ve, 16)];
 //calcular_tempo_enriquecimento_gama(valor_referencia + 100, correcao_aquecimento + 100, correcao_O2 + 100, correcao_temperatura_ar + 100, correcao_barometrica + 100);//100 equivale a sem mudanças
          // Calcula o tempo de injeção ajustado
          float tempo_pulso = tempo_pulso_ve(dreq_fuel / 1000, VE);
          calcula_enriquecimento_aceleracao();
          unsigned long incremento_percentual = round(tempo_pulso * (tps_dot_porcentagem_aceleracao / 100.0));
          tempo_injecao = tempo_pulso + tempo_abertura_injetor + incremento_percentual;
          // tempo_injecao = round(tempo_pulso);
          
          // tempo_atual = micros();
             
// if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0 ){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
// for (int i = 0; i < qtd_cilindro; i++)
// {
//   if (grau_pms <= 120) {
//     if (grau_pms < 60 || rpm > 3000) {
//         ajuste_pms = 180;
//     }else{
//       ajuste_pms = 0;
//     } 
//   }else{
//     ajuste_pms = 0;
//   }

// calcula_grau_injetor(i);
// calcula_grau_ignicao(i);
// }
// }
    leitura_entrada_dados_serial(); 
  // verifica se já passou o intervalo de tempo
  if (millis() - ultima_execucao >= intervalo_execucao){     
  rpm_anterior = rpm; 
  //Serial.println(analogRead(pino_sensor_tps));
  // Exibe a taxa de mudança do TPS (TPSDot) no monitor serial
  envia_dados_tempo_real(1);
  temperatura_motor = temperatura_clt();
  protege_ignicao_injecao();
  //Serial.println(qtd_loop*(1000/intervalo_execucao)); 
  //Serial.println(freeMemory()); 
   // atualiza o tempo da última execução
   ultima_execucao = millis();
   qtd_loop = 0;
  // Serial.print(tempo_decorrido_codigo);
  // Serial.println(REQ_FUEL);
  // Serial.println(dreq_fuel);
  // Serial.println(tempo_injecao);

  // Serial.println(rpm_anterior);
  }
  //  tempo_final_codigo = micros(); // Registra o tempo final  
  //  tempo_decorrido_codigo = tempo_final_codigo - tempo_inicial_codigo; 
  //  enviar_byte_serial(tempo_decorrido_codigo, 2);
}
