// Largura base do pulso de injecao, em microssegundos.
// REQ_FUEL * (VE/100) * 1000, em aritmetica inteira.
unsigned long tempo_pulso_ve(unsigned long REQ_FUEL, int VE) {
    if (VE < 0) {
        return 0;
    }
    return (REQ_FUEL * (unsigned long)VE * 10UL);
}

// (tempo_enriquecimento_gama removida: era codigo morto - quem e chamado e a
// enriquecimento_gama de enriquecimento_gama.h, de nome parecido)

static inline byte indice_pino_injecao(int i) {
  // Mesmo sistema Wasted Spark da ignição, só que para injeção (Semi-Sequencial no Vira)
  // Faz mapear o índice cíclico matemático para dentro dos injetores fisicamente instalados.
  if (local_rodafonica == 2 && modo_injecao == 2) { // modo_injecao 2 = Semi-sequencial
      return (byte)(i % (qtd_cilindro / 2));
  }

  // Comportamento do fase/comando que era antigo
  if (local_rodafonica == 1 && i >= (qtd_cilindro / 2)) {
    return (byte)(i - (qtd_cilindro / 2));
  }
  
  return (byte)i;
}

// Quantos EVENTOS de injecao existem por ciclo do sensor. Isso depende do
// motor (cilindros, 2 ou 4 tempos) e de onde a roda fonica esta - NAO da
// quantidade de injetores. Quantos injetores sao acionados em cada evento e
// outra coisa, decidida em ligar_injetor pelo modo e por numero_injetor.
//
// A distincao importa no monoponto: um motor de 6 cilindros com um unico bico
// ainda precisa pulsar a cada cilindro, e nao uma vez por volta.
//
// O modo pareado tinha aqui um "return 1", introduzido como otimizacao de
// agendamento (76c1509) sob a ideia de que um canal logico bastaria por os
// injetores acionarem juntos. Estava errado: os eventos nao sao redundantes,
// sao pulsos de injecao distintos. Com REQ_FUEL calculado para N esguichos por
// ciclo (ver a calculadora dreqfuel, que divide por numsquirts), disparar 2
// vezes por ciclo em vez de 6 entrega um TERCO do combustivel previsto - falha
// de mistura pobre, nao apenas diferenca de comportamento.
static inline byte quantidade_eventos_injecao_por_ciclo_sensor() {
  if (local_rodafonica == 2 && tipo_motor == 4 && qtd_cilindro > 1) {
    return qtd_cilindro / 2;
  }
  return qtd_cilindro;
}

static inline bool existe_outro_evento_injecao_ativo(int evento_atual) {
  byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
  for (int i = 0; i < eventos_injecao; i++) {
    if (i != evento_atual && captura_req_fuel[i] == true && inj_acionado[i] == true) {
      return true;
    }
  }
  return false;
}

static inline bool existe_outro_evento_injecao_ativo_no_pino(int evento_atual, byte pino) {
  byte eventos_injecao = quantidade_eventos_injecao_por_ciclo_sensor();
  for (int i = 0; i < eventos_injecao; i++) {
    if (i != evento_atual && captura_req_fuel[i] == true && inj_acionado[i] == true &&
        indice_pino_injecao(i) == pino) {
      return true;
    }
  }
  return false;
}

static inline int calcular_angulo_injecao_indice(int i) {
      int angulo_base_injecao = ajuste_pms + grau_pms - offset_referencia_roda_fonica_graus() + (grau_entre_cada_cilindro * i) - grau_fechamento_injetor;
      while (angulo_base_injecao < 0) {
        angulo_base_injecao += 360;
      }
      while (angulo_base_injecao >= 360) {
        angulo_base_injecao -= 360;
      }
      if (angulo_base_injecao == 0) {
        angulo_base_injecao = 1;
      }
      return angulo_base_injecao;
}

void calcula_grau_injetor(int i){
if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false)){
      int angulo_base_injecao = calcular_angulo_injecao_indice(i);
      tempo_proxima_injecao[i] = (unsigned long)angulo_base_injecao * tempo_cada_grau;
    }
}
void ligar_injetor(int i){
    if ((captura_req_fuel[i] == false) && (inj_acionado[i] == false) &&
        revolucoes_sincronizada >= 1 && status_corte == 0 && !limpeza_afogamento_ativa){
        if(modo_injecao == 1 || tipo_acionamento_injetor == 1){
          for (int j = 0; j < numero_injetor; j++){
          digitalWrite(injecao_pins[j], HIGH);
          }
          captura_req_fuel[i] = true;
          tempo_percorrido_inj[i] = micros();
          inj_acionado[i] = true;
        }else{
          byte pino = indice_pino_injecao(i);
          captura_req_fuel[i] = true;
          tempo_percorrido_inj[i] = micros();
          digitalWrite(injecao_pins[pino], HIGH);
          inj_acionado[i] = true;
        }
  }
}

void desligar_injetor(int i){
  if (captura_req_fuel[i] == true && inj_acionado[i] == true){
          captura_req_fuel[i] = false;
          inj_acionado[i] = false;
          if (modo_injecao == 1 || tipo_acionamento_injetor == 1){
            if (local_rodafonica != 2 || !existe_outro_evento_injecao_ativo(i)) {
              for (int j = 0; j < numero_injetor; j++){
                digitalWrite(injecao_pins[j], LOW);
              }
            }
          } else {
            byte pino = indice_pino_injecao(i);
            if (local_rodafonica != 2 || !existe_outro_evento_injecao_ativo_no_pino(i, pino)) {
              digitalWrite(injecao_pins[pino], LOW);
            }
          }
  }
}

