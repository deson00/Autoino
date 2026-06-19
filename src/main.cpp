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
#include <avanco_temperatura.h>

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
  const unsigned long TIMEOUT_SEM_PULSO_MIN_US = 250000;
  const unsigned long TIMEOUT_SEM_PULSO_MAX_US = 1500000;
  const unsigned int RPM_MAX_VALIDO = 12000;          // rejeita leituras fora de faixa

  static unsigned int rpm_filtrado = 0;
  static unsigned long tempo_volta_valido_us = 0;
  static unsigned long ultimo_pulso_observado_us = 0;
  static byte timeout_consecutivo = 0;

  unsigned long tempo_atual_local = micros();
  unsigned long tempo_volta_snapshot;
  unsigned long ultimo_pulso_snapshot;

  noInterrupts();
  tempo_volta_snapshot = tempo_total_volta_completa;
  tempo_total_volta_completa = 0;
  ultimo_pulso_snapshot = ultimo_pulso_rpm_us;
  interrupts();

  if (ultimo_pulso_snapshot != ultimo_pulso_observado_us) {
    ultimo_pulso_observado_us = ultimo_pulso_snapshot;
    timeout_consecutivo = 0;
  }

  if (tempo_volta_snapshot > 0 && tempo_volta_snapshot < 5000000UL) {
    tempo_volta_valido_us = tempo_volta_snapshot;
  }

  unsigned long timeout_sem_pulso_us = TIMEOUT_SEM_PULSO_MIN_US;
  if (tempo_volta_valido_us > 0) {
    timeout_sem_pulso_us = tempo_volta_valido_us << 1;
    if (timeout_sem_pulso_us < TIMEOUT_SEM_PULSO_MIN_US) {
      timeout_sem_pulso_us = TIMEOUT_SEM_PULSO_MIN_US;
    } else if (timeout_sem_pulso_us > TIMEOUT_SEM_PULSO_MAX_US) {
      timeout_sem_pulso_us = TIMEOUT_SEM_PULSO_MAX_US;
    }
  }

  if ((tempo_atual_local - ultimo_pulso_snapshot) > timeout_sem_pulso_us) {
    if (timeout_consecutivo < 255) {
      timeout_consecutivo++;
    }
    if (timeout_consecutivo >= 2) {
      rpm_filtrado = 0;
      rpm = 0;
    }
    tempo_inicial_rpm = tempo_atual_local;
    return;
  }

  timeout_consecutivo = 0;

  if (tempo_volta_snapshot > 0) {
    unsigned long rpm_calculado = 60000000UL / tempo_volta_snapshot;

    if (local_rodafonica == 1) {
      rpm_calculado <<= 1;
    }

    if (rpm_calculado > 0 && rpm_calculado < RPM_MAX_VALIDO) {
      unsigned int rpm_alvo = (unsigned int)rpm_calculado;

      if (rpm_filtrado == 0) {
        rpm_filtrado = rpm_alvo;
      } else {
        unsigned int variacao_maxima = (rpm_filtrado >> 2) + (rpm_filtrado >> 3) + 30; // ~37.5% + margem fixa
        int diferenca = (int)rpm_alvo - (int)rpm_filtrado;

        if (diferenca > (int)variacao_maxima) {
          diferenca = (int)variacao_maxima;
        } else if (diferenca < -((int)variacao_maxima)) {
          diferenca = -((int)variacao_maxima);
        }

        rpm_filtrado = (unsigned int)((int)rpm_filtrado + ((diferenca * 5) >> 3)); // IIR ~0.625 (mais responsivo)
      }

      rpm = rpm_filtrado;
    }
  }

  tempo_inicial_rpm = tempo_atual_local;
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
  setupTimer1();
  Serial.begin(9600);
  delay(200);
  tempo_inicial_rpm = micros();
  ultimo_pulso_rpm_us = tempo_inicial_rpm;
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
    int leitura_adc = analogRead(pino_sensor_o2);

    // tensão em milivolts (0 a 5000 mV)
    int tensao_o2 = (leitura_adc * 5000) / 1023;
    valor_o2 = tensao_o2;

    int lambda_x1000 = 1000;
    if (tipo_sonda_o2) {
      // Wideband 0.2V..4.8V -> lambda 0.59..1.10 (x1000)
      lambda_x1000 = 590 + ((long)(tensao_o2 - 200) * (1100 - 590)) / (4800 - 200);
      if (lambda_x1000 < 590) {
        lambda_x1000 = 590;
      } else if (lambda_x1000 > 1100) {
        lambda_x1000 = 1100;
      }
    } else {
      // Narrowband: aproximação para leitura de lambda em torno do estequiométrico
      // 100mV (lean) -> 1.10, 900mV (rich) -> 0.90
      lambda_x1000 = 1100 - ((long)(tensao_o2 - 100) * (1100 - 900)) / (900 - 100);
      if (lambda_x1000 < 900) {
        lambda_x1000 = 900;
      } else if (lambda_x1000 > 1100) {
        lambda_x1000 = 1100;
      }
    }

    sonda_o2 = lambda_x1000;
    
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
         
    bool avanco_baseado_em_tabela = false;

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
      avanco_baseado_em_tabela = true;
      dwell_bobina = dwell_funcionamento * 1000ul;
      status_corte = 0;
    }
    else{
      grau_avanco = matriz_avanco[procura_indice(valor_referencia_busca_avanco, vetor_map_tps, 16)][procura_indice(rpm, vetor_rpm, 16)];
      avanco_baseado_em_tabela = true;
      dwell_bobina = dwell_funcionamento * 1000ul;
      status_corte = 0;
    }

    if (usar_avanco_temperatura == 1 && avanco_baseado_em_tabela && status_corte == 0) {
      byte correcao_avanco_temp = avanco_por_temperatura((float)temperatura_motor);
      unsigned int grau_corrigido = (unsigned int)grau_avanco + (unsigned int)correcao_avanco_temp;
      if (grau_corrigido > 120U) {
        grau_corrigido = 120U;
      }
      grau_avanco = (byte)grau_corrigido;
    }

          VE = matriz_ve[procura_indice(valor_referencia_busca_tempo_injecao, vetor_map_tps_ve, 16)][procura_indice(rpm, vetor_rpm_ve, 16)];
          // Calcula o tempo de injeção ajustado
          float tempo_pulso = tempo_pulso_ve(dreq_fuel / 1000, VE);
          //exemplo de entrada calcular_tempo_enriquecimento_gama(tempo_base_injecao, correcao_aquecimento, correcao_O2, correcao_temperatura_ar, correcao_barometrica) 
          tempo_injecao = enriquecimento_gama(tempo_pulso, enriquecimento_temperatura(temperatura_motor), 100, 100, 100);   
          calcula_enriquecimento_aceleracao(tempo_pulso);
          tempo_injecao = tempo_pulso + tempo_abertura_injetor + incremento_aceleracao - decremento_desaceleracao;
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
