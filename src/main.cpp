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
// #include <decoder_padrao_melhorado.h>
#include <injecao.h>
#include <ignicao.h>
#include <timer.h>
#include <timer2.h>
#include <enriquecimento_aceleracao.h>
#include <enriquecimento_gama.h>
#include <enriquecimento_temperatura.h>

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
// void calcularRPM() {
//     static unsigned long ultimo_calculo_rpm = 0;
    
//     // Calcula RPM em um intervalo regular, por exemplo, a cada 100ms
//     if (micros() - ultimo_calculo_rpm > 100000) { 
//         ultimo_calculo_rpm = micros();

//         unsigned long revolucoes_local;
//         unsigned long tempo_decorrido_local;
        
//         // --- TÉCNICA DE LEITURA SEGURA (SEM INTERRUPÇÕES DESLIGADAS) ---
//         // Copia a variável 'qtd_revolucoes' até ter certeza que ela não mudou
//         // durante a leitura. Isso evita a condição de corrida.
//         do {
//             revolucoes_local = qtd_revolucoes;
//             tempo_decorrido_local = micros();
//         } while (revolucoes_local != qtd_revolucoes);

//         // Se o valor do loop foi o mesmo da interrupção, faz a cópia do tempo.
//         tempo_decorrido_local = tempo_decorrido_local - tempo_inicial_rpm;
//         tempo_inicial_rpm = micros();

//         // Agora, com dados consistentes, faça o cálculo.
//         if (revolucoes_local > 0) {
//             float delta_segundos = (float)tempo_decorrido_local / 1000000.0;
//             float rpm_calculado = (revolucoes_local * 60.0) / delta_segundos;
            
//             // Ajuste para roda no comando, se aplicável
//             if (local_rodafonica == 1) {
//                 rpm_calculado = rpm_calculado * 2; 
//             }
            
//             // Filtro para estabilizar a leitura de RPM
//             if (rpm_calculado < 10000) {
//                  rpm = rpm_calculado;
//             }
//         }
//     }
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
   // Inicializa decoder melhorado
  // inicializar_decoder_roda_fonica();
  // inicializar_decoder_otimizado();
  // Para começar com o original:
// inicializar_decoder_roda_fonica();

// Depois testar o otimizado:
// inicializar_decoder_otimizado();

// Para alternar em tempo real:
// usar_decoder_original();    // volta ao original
// usar_decoder_otimizado();   // usa o melhorado
  
  sei(); // Habilita interrupções globais
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
    float leitura_adc = analogRead(pino_sensor_o2);
    float tensao_o2   = leitura_adc * (5.0 / 1023.0); 
    // float tensao_mV   = tensao_o2 * 1000.0;

    if (tipo_sonda_o2 == 0)
    {
        // --- Narrow Band ---
        if (tensao_o2 < 0.45) 
            sonda_o2 = 0;   // Pobre
        else 
            sonda_o2 = 1;   // Rica
    }
    else
    {
        // --- Wideband (Faixa 1: 0,59 a 1,10 Lambda) ---
        if (tensao_o2 < 0.2) tensao_o2 = 0.2;
        if (tensao_o2 > 4.8) tensao_o2 = 4.8;

        // Conversão linear para Lambda
        sonda_o2 = 0.59 + ( (tensao_o2 - 0.2) * (1.10 - 0.59) / (4.8 - 0.2) );
        sonda_o2 = sonda_o2 * 1000; // Multiplica por 1000 para evitar uso de float em outras partes}
    }
    
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
          // Calcula o tempo de injeção ajustado
          float tempo_pulso = tempo_pulso_ve(dreq_fuel / 1000, VE);
          calcula_enriquecimento_aceleracao();
          unsigned long incremento_percentual = round(tempo_pulso * (tps_dot_porcentagem_aceleracao / 100.0));
          tempo_injecao = tempo_pulso + tempo_abertura_injetor + incremento_percentual;
          //calcular_tempo_enriquecimento_gama(tempo_base_injecao, correcao_aquecimento, correcao_O2, correcao_temperatura_ar, correcao_barometrica) 
          tempo_injecao = enriquecimento_gama(tempo_injecao, enriquecimento_temperatura(temperatura_motor, temperatura_trabalho, correcao_maxima_temperatura), 100, 100, 100);   
          if(rpm < rpm_partida){
            // Aplicando o acréscimo de injeção na partida
            tempo_injecao = tempo_injecao + (tempo_injecao * (acrescimo_injecao_partida / 100.0));
          }
          // tempo_injecao = round(tempo_pulso);
          if(status_primeira_injecao == false){ 
            for (int j = 0; j < numero_injetor; j++){
              digitalWrite(injecao_pins[j], 1);
            }
            delay(tempo_primeira_injecao);
            for (int j = 0; j < numero_injetor; j++){
              digitalWrite(injecao_pins[j], 0);
            }
            status_primeira_injecao = true;
          }
          
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
