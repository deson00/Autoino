// Largura base do pulso de injecao, em microssegundos: REQ_FUEL * VE / 100.
// req_fuel_us ja vem em MICROSSEGUNDOS (dreq_fuel = 3600 significa 3,6ms).
//
// Antes o chamador fazia "dreq_fuel / 1000" para converter para ms, mas
// dreq_fuel e int e essa e uma divisao INTEIRA: 3600 virava 3 em vez de 3,6, e
// o motor recebia ~17% menos combustivel do que a tela configurava. Trabalhar
// direto em us elimina a conversao e o truncamento de uma vez.
unsigned long tempo_pulso_ve(unsigned long req_fuel_us, int VE) {
    if (VE <= 0) {
        return 0;
    }
    return (req_fuel_us * (unsigned long)VE) / 100UL;
}

// VE interpolada entre as 4 celulas em volta do ponto de operacao, como a
// Speeduino faz: primeiro pelo RPM na linha de carga de baixo e na de cima,
// depois pela carga entre as duas. Ex.: 1100 rpm entre 1000 (VE 50) e 1500
// (VE 60) da 52 - 1% de VE a cada 50 rpm.
//
// Antes a VE vinha da celula mais proxima (procura_indice): de 750 a 1250 rpm
// era 50 cravado e a 1251 pulava para 60 de uma vez, um degrau de 20% de
// combustivel que o motor sentia ao cruzar a metade entre duas colunas.
//
// Reaproveita o que o avanco linear ja usa: procura_indice_inferior (nunca
// devolve o ultimo indice, entao i+1 e seguro) e busca_linear (interpola sem
// extrapolar - fora do eixo fica no valor da ponta, igual a celula de antes).
static int ve_interpolada(int carga, int rpm_atual) {
  int i = procura_indice_inferior(carga, vetor_map_tps_ve, 16);
  int j = procura_indice_inferior(rpm_atual, vetor_rpm_ve, 16);
  int r0 = vetor_rpm_ve[j];
  int r1 = vetor_rpm_ve[j + 1];
  int ve_baixo = busca_linear(rpm_atual, r0, matriz_ve[i][j], r1, matriz_ve[i][j + 1]);
  int ve_cima = busca_linear(rpm_atual, r0, matriz_ve[i + 1][j], r1, matriz_ve[i + 1][j + 1]);
  return busca_linear(carga, vetor_map_tps_ve[i], ve_baixo, vetor_map_tps_ve[i + 1], ve_cima);
}

// (tempo_enriquecimento_gama removida: era codigo morto - quem e chamado e a
// enriquecimento_gama de enriquecimento_gama.h, de nome parecido)

static inline byte indice_pino_injecao(int i) {
  if (sensor_sem_falha()) {
    return 0; // monoponto, ou pareado que aciona todos via numero_injetor
  }
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
  if (modo_injecao == 0) {
    // Injecao DESLIGADA: motor a carburador, ou conversao so de ignicao.
    // Zero eventos significa que nada e agendado, nada e varrido nas ISRs do
    // Timer1 e nenhum bico e acionado - todos os lacos deste arquivo e de
    // timer.h sao "for (i = 0; i < eventos_injecao; i++)".
    //
    // Nao e so economia de trabalho inutil: numa 60-2 com 6 cilindros em
    // pareado a injecao gera 3 eventos por volta, ou seja 6 dos 8 eventos de
    // Timer1 disputando o agendador com a ignicao em RPM alto.
    return 0;
  }
  if (sensor_sem_falha()) {
    return 1; // um evento por pulso, pelo mesmo motivo da ignicao
  }
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
      // A injecao nao tem o caso especial da centelha perdida, entao vai direto
      // pela tabela - que com ela desligada e a mesma multiplicacao de antes.
      int angulo_base_injecao = ajuste_pms + grau_pms - offset_referencia_roda_fonica_graus() + separacao_evento_base(i) - grau_fechamento_injetor;
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
        revolucoes_sincronizada >= 1 && status_corte == 0 && !limpeza_afogamento_ativa &&
        !TESTE_COMPRESSAO_SUPRIME){
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

// Mesma correcao de desligar_dwell: guarda pelo estado fisico (inj_acionado) e
// limpa ele primeiro, para nao existir janela em que a varredura pede o
// desligamento e esta funcao o ignora.
void desligar_injetor(int i){
  if (inj_acionado[i] == true){
          inj_acionado[i] = false;
          captura_req_fuel[i] = false;
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

