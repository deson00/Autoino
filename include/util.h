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
int busca_linear(int rpm_atual, int rpm_minimo, int grau_minimo, int rpm_maximo, int grau_maximo) {
  // Cálculo da proporção
  float proporcao = float(rpm_atual - rpm_minimo) / float(rpm_maximo - rpm_minimo);
  // Mapeamento linear
  int grau = proporcao * (grau_maximo - grau_minimo) + grau_minimo; 
  return grau;
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

static inline int offset_referencia_roda_fonica_graus() {
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

float calculateBeta(float ntcResistance1, float ntcTemperature1, float ntcResistance2, float ntcTemperature2) {
  float T1 = ntcTemperature1 + 273.15;   // converte a temperatura em Celsius para Kelvin
  float T2 = ntcTemperature2 + 273.15;
  float beta = log(ntcResistance2/ntcResistance1) / ((1/T2) - (1/T1));   // aplica a equação do coeficiente beta
  return beta;
}

float calculateTemperature(float ntcResistance, float ntcBeta, float ntcReferenceResistance, float ntcReferenceTemperature) {
  float steinhart;
  steinhart = log(ntcResistance/ntcReferenceResistance) / ntcBeta;     // parte da equação de Steinhart-Hart
  steinhart += 1.0 / (ntcReferenceTemperature + 273.15);                    // adiciona a temperatura de referência em Kelvin
  steinhart = 1.0 / steinhart;                                             // inverte a equação
  steinhart -= 273.15;                                                     // converte para Celsius
  return steinhart;
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
void escrever_16bits_eeprom(int endereco, uint16_t valor) {
    EEPROM.write(endereco, valor & 0xFF);        // LSB
    EEPROM.write(endereco + 1, (valor >> 8) & 0xFF); // MSB
}

uint16_t ler_16bits_eeprom(int endereco) {
    uint8_t lowByte = EEPROM.read(endereco);
    uint8_t highByte = EEPROM.read(endereco + 1);
    return (highByte << 8) | lowByte;
}

void escrever_8bits_eeprom(int endereco, uint8_t valor) {
    EEPROM.write(endereco, valor);
}

uint8_t ler_8bits_eeprom(int endereco) {
    return EEPROM.read(endereco);
}
