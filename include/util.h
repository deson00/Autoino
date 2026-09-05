//util destinado a funçoes de uso geral 

int procura_indice(int value, int *arr, int size){
  int index = 0;
  int closest = abs(arr[0] - value);
  for (int i = 1; i < size; i++){
    int diff = abs(arr[i] - value);
    if (diff < closest){
      closest = diff;
      index = i;
    }
    if (arr[i] == value) {
      return i;
    }
  }
  return index;
}
// Interpolacao linear entre dois pontos da tabela de avanco, em aritmetica
// inteira: multiplica antes de dividir para nao perder resolucao. Trunca em
// direcao a zero, como a versao anterior em float ja fazia ao converter para
// int no retorno.
// Interpola o avanco entre dois pontos da tabela. INTERPOLA, nunca extrapola:
// o resultado e limitado aos dois graus da tabela.
//
// Sem esse limite, um rpm fora do trecho coberto pelo eixo faz a reta seguir
// alem do ultimo ponto configurado e devolver um avanco que ninguem escreveu
// na tabela. Num sistema de ignicao isso nao e um numero errado qualquer -
// avanco demais e detonacao.
int busca_linear(int rpm_atual, int rpm_minimo, int grau_minimo, int rpm_maximo, int grau_maximo) {
  int32_t faixa_rpm = (int32_t)rpm_maximo - rpm_minimo;
  if (faixa_rpm == 0) {
    return grau_maximo;
  }
  int32_t numerador = (int32_t)(grau_maximo - grau_minimo) * ((int32_t)rpm_atual - rpm_minimo);
  int32_t resultado = (int32_t)grau_minimo + (numerador / faixa_rpm);

  int32_t piso = (grau_minimo < grau_maximo) ? grau_minimo : grau_maximo;
  int32_t teto = (grau_minimo < grau_maximo) ? grau_maximo : grau_minimo;
  if (resultado < piso) {
    resultado = piso;
  } else if (resultado > teto) {
    resultado = teto;
  }
  return (int)resultado;
}

// Indice da celula INFERIOR, para quem vai interpolar entre i e i+1.
//
// procura_indice nao serve aqui, por dois motivos:
//
// 1. Ela devolve o ponto mais PROXIMO. Com o valor na metade de cima de uma
//    celula ela escolhe o ponto de CIMA, e a interpolacao seguinte extrapola
//    para tras a partir dele. Em tabela suave o erro e pequeno; onde a tabela
//    tem degrau, nao e.
// 2. Ela pode devolver o ULTIMO indice, e o chamador entao le vetor[size] e
//    matriz[linha][size] - fora dos limites do array. O avanco resultante e
//    lixo de memoria vizinha.
//
// So percorre o trecho crescente do inicio do eixo: eixo com menos de 16
// pontos configurados fica com zeros no fim, e zero nao e ponto valido.
// O retorno e sempre <= (ultimo ponto valido - 1), entao i+1 e seguro.
static int procura_indice_inferior(int valor, const int *vetor, int size) {
  int ultimo = 0;
  while (ultimo + 1 < size && vetor[ultimo + 1] > vetor[ultimo]) {
    ultimo++;
  }
  if (ultimo < 1) {
    return 0; // eixo degenerado: nao ha par de pontos para interpolar
  }

  int limite = ultimo - 1;
  int i = 0;
  while (i < limite && vetor[i + 1] <= valor) {
    i++;
  }
  return i;
}

static const int MARGEM_IGNICAO_FIM_CICLO_GRAUS = 1;

static inline int graus_avanco_para_referencia_sensor(int graus_virabrequim) {
  // Quando a roda esta no comando, 360 graus do sensor equivalem a 720 no virabrequim.
  if (local_rodafonica == 1) {
    return graus_virabrequim / 2;
  }
  return graus_virabrequim;
}

static inline int normalizar_angulo_minimo_zero(int angulo) {
  // Divisao/modulo de 16 bits sao caras no AVR (sem divisor em hardware).
  // O angulo aqui nunca fica muito longe de [0,360), entao subtrair em loop
  // e bem mais barato que "% 360" nesse caminho critico (roda por dente).
  while (angulo < 0) {
    angulo += 360;
  }
  while (angulo >= 360) {
    angulo -= 360;
  }
  return angulo;
}

// Sensor SEM dente de falha: cada pulso ja e uma referencia angular por si so.
// Cobre o distribuidor com sensor hall (um pulso por cilindro, tipo Opala) e o
// volante com um unico dente (motos, tipo Titan). tipo_ignicao ja existia na
// configuracao com esse significado documentado, mas nunca era consultado.
static inline bool sensor_sem_falha() {
  return tipo_ignicao == 2;
}

static inline int offset_referencia_roda_fonica_graus() {
  if (sensor_sem_falha()) {
    return 0; // nao ha gap: o angulo e medido a partir do proprio pulso
  }
  if ((local_rodafonica == 1 || local_rodafonica == 2) && grau_cada_dente > 0) {
    return ((int)qtd_dente_faltante + 1) * (int)grau_cada_dente;
  }
  return 0;
}

// Espacamento angular entre eventos de cilindros consecutivos. Com o sensor
// no virabrequinho, isso depende de quantos graus de virabrequim formam um
// ciclo completo de combustao: 720 graus (2 voltas) num motor 4 tempos,
// 360 graus (1 volta) num motor 2 tempos. Com o sensor no comando, o proprio
// sensor ja gira 1 volta por ciclo, entao sempre 360 graus, nos dois casos.
// Retorna int, nao byte: o resultado passa de 255 em motores de poucos
// cilindros e era truncado silenciosamente. Num 4 tempos com sensor no
// virabrequim o ciclo tem 720 graus, entao 2 cilindros dao 360 (virava 104) e
// 1 cilindro da 720 (virava 208), jogando ignicao e injecao em angulos sem
// relacao com o motor. O compilador ja avisava disso no "return 360" abaixo.
// Nao afeta 3 cilindros ou mais, onde o valor cabe em um byte.
static inline int calcular_grau_entre_cada_cilindro() {
  if (qtd_cilindro < 1) {
    return 360;
  }
  if (local_rodafonica == 2) {
    int graus_ciclo_completo = (tipo_motor == 2) ? 360 : 720;
    return graus_ciclo_completo / qtd_cilindro;
  }
  return 360 / qtd_cilindro;
}

// log2(valor) em ponto fixo Q16. Substitui o log() de ponto flutuante da
// biblioteca padrao, que sozinho arrastava ~1150 bytes de rotinas float para
// o binario (__divsf3x, __mulsf3x, __addsf3x, __fp_powser e conversoes).
//
// Algoritmo classico: separa a parte inteira pela posicao do bit mais
// significativo, normaliza a mantissa para [1,2) em Q15 e extrai 16 bits de
// fracao elevando ao quadrado repetidamente. Cada quadrado vale um bit.
// Sem estouro: a mantissa fica sempre abaixo de 65536, e 65535^2 ainda cabe
// em 32 bits.
static uint32_t log2_q16(uint32_t valor) {
  if (valor == 0) {
    return 0;
  }

  uint8_t expoente = 0;
  for (uint32_t v = valor; v > 1; v >>= 1) {
    expoente++;
  }

  uint32_t mantissa;
  if (expoente > 15) {
    mantissa = valor >> (expoente - 15);
  } else {
    mantissa = valor << (15 - expoente);
  }

  uint32_t fracao = 0;
  for (uint8_t i = 0; i < 16; i++) {
    mantissa = (mantissa * mantissa) >> 15;
    fracao <<= 1;
    if (mantissa >= 65536UL) {
      mantissa >>= 1;
      fracao |= 1;
    }
  }

  return ((uint32_t)expoente << 16) | fracao;
}

// Temperatura de um NTC a partir da resistencia, por interpolacao entre os
// dois pontos de calibracao - sem precisar do coeficiente beta.
//
// A equacao de beta diz  1/T = 1/T1 + ln(R/R1)/beta, e o proprio beta vale
// ln(R2/R1)/(1/T2 - 1/T1). Substituindo um no outro, o beta se cancela:
//
//     1/T = 1/T1 + k * (1/T2 - 1/T1),  com  k = ln(R/R1) / ln(R2/R1)
//
// Ou seja, e uma interpolacao linear no inverso da temperatura. E como k e uma
// RAZAO entre logaritmos, a base se cancela tambem - da pra usar log2 inteiro
// no lugar do logaritmo natural. O resultado e matematicamente o mesmo da
// formula anterior, so que sem ponto flutuante.
//
// Precisao: o erro de log2_q16 se traduz em menos de 0,2 C na faixa util, o
// que e irrelevante aqui porque a temperatura so alimenta duas tabelas de 5
// pontos (enriquecimento_temperatura e avanco_por_temperatura).
//
// ALTERNATIVA CONSIDERADA, para retomar se um dia precisar de mais precisao ou
// de suportar sensores nao-NTC: mandar da UI uma tabela ADC -> temperatura ja
// calculada (como a Speeduino faz), deixando aqui so a interpolacao. Custa um
// bloco novo de protocolo, espaco na EEPROM e acoplamento entre versoes de UI
// e firmware, por isso ficou de fora agora.
static int calcular_temperatura_ntc(uint32_t resistencia,
                                    int resistencia_ref1, int temperatura_ref1,
                                    int resistencia_ref2, int temperatura_ref2) {
  if (resistencia == 0 || resistencia_ref1 <= 0 || resistencia_ref2 <= 0) {
    return 250;
  }

  int32_t log_r = (int32_t)log2_q16(resistencia);
  int32_t log_r1 = (int32_t)log2_q16((uint32_t)resistencia_ref1);
  int32_t log_r2 = (int32_t)log2_q16((uint32_t)resistencia_ref2);

  int32_t denominador = log_r2 - log_r1;
  if (denominador == 0) {
    return 250;
  }

  // k em Q10. Q10 e o maior expoente seguro aqui: log_r cabe em ~1,1e6 para
  // resistencias de ate 100k, e 1,1e6 << 10 ainda cabe em int32.
  int32_t k_q10 = ((log_r - log_r1) << 10) / denominador;

  // Inverso da temperatura em milionesimos de Kelvin.
  int32_t inv_t1 = 1000000L / ((int32_t)temperatura_ref1 + 273L);
  int32_t inv_t2 = 1000000L / ((int32_t)temperatura_ref2 + 273L);
  int32_t inv_t = inv_t1 + ((k_q10 * (inv_t2 - inv_t1)) >> 10);
  if (inv_t <= 0) {
    return 250;
  }

  return (int)((1000000L / inv_t) - 273L);
}


void sort(int arr[], int n) {
 for (int i = 0; i < n-1; i++) {
    for (int j = 0; j < n-i-1; j++) {
      if (arr[j] > arr[j+1]) {
        // Troca arr[j] e arr[j+1]
        int temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;
      }
    }
 }
}

void enviar_byte_serial(int valor, int tamanho) {
  if (tamanho == 1) {
    // Verifica se o valor é um caractere
    if (valor >= 0 && valor <= 255) {
      // Se for um caractere, envia o byte diretamente
      Serial.write((char)valor);
    } else {
      // Se não for um caractere, envia apenas o byte menos significativo
      char byteBaixo = valor & 0xFF;  // Os 8 bits menos significativos
      Serial.write(byteBaixo);
    }
  } else if (tamanho == 2) {
    // Divide o valor em dois bytes
    char byteBaixo = valor & 0xFF;        // Os 8 bits menos significativos
    char byteAlto = (valor >> 8) & 0xFF;  // Os 8 bits mais significativos

    // Envia os dois bytes pela porta serial
    Serial.write(byteBaixo);
    Serial.write(byteAlto);
  }
}

// FUNÇÕES AUXILIARES PARA EEPROM
// EEPROM.update, nao EEPROM.write.
//
// update le o byte antes e so grava se mudou; write grava sempre, e no AVR
// cada gravacao custa 3,3ms de CPU parada. Quem passa por aqui sao as duas
// tabelas grandes (avanco e VE): 304 bytes cada, 608 no total, regravados
// inteiros a cada clique em "Gravar Modificacoes" mesmo com uma unica celula
// alterada. Sao ~2 SEGUNDOS com o loop() parado - e loop parado e
// processar_agendamento_pendente() sem rodar, ou seja nenhuma ignicao
// agendada. O motor apagava ao salvar a tabela com o carro ligado.
//
// O resto do projeto ja usava update (97 chamadas nos blocos de
// configuracao); a correcao nunca tinha chegado nestes tres pontos, que sao
// justamente os que gravam mais bytes.
//
// Ganho secundario: EEPROM tem vida util de ~100 mil gravacoes por celula.
// Regravar 608 bytes a cada salvamento gastava a memoria a toa.
void escrever_16bits_eeprom(int endereco, uint16_t valor) {
    EEPROM.update(endereco, valor & 0xFF);        // LSB
    EEPROM.update(endereco + 1, (valor >> 8) & 0xFF); // MSB
}

uint16_t ler_16bits_eeprom(int endereco) {
    uint8_t lowByte = EEPROM.read(endereco);
    uint8_t highByte = EEPROM.read(endereco + 1);
    return (highByte << 8) | lowByte;
}

void escrever_8bits_eeprom(int endereco, uint8_t valor) {
    EEPROM.update(endereco, valor); // ver o comentario em escrever_16bits_eeprom
}

uint8_t ler_8bits_eeprom(int endereco) {
    return EEPROM.read(endereco);
}
