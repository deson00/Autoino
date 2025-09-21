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

int freeMemory() {
  int size = 1024; // Tamanho inicial da alocação
  byte *buf;
  
  while ((buf = (byte *) malloc(--size)) == NULL); // Aloca até que falhe
  
  free(buf); // Libera a memória alocada
  return size;
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
