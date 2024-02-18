#include <Arduino.h>
#include <EEPROM.h>
//#define Uno   //Descomente essa linha caso utilizar um Arduino UNO ou Nano
#define Mega  //Descomente essa linha caso utilizar um Arduino Mega

#ifdef Uno
#define pino_sensor_roda_fonica 2
#define pino_sensor_fase 3
#define pino_sensor_map A0
#define pino_sensor_clt A2
int ign1 = 4;
int ign2 = 5;
int ign3 = 6;
int ign4 = 7;
#endif
#ifdef Mega
#define pino_sensor_roda_fonica 19
#define pino_sensor_fase 18
#define pino_sensor_map A3
#define pino_sensor_clt A1
int ign1 = 40;
int ign2 = 38;
int ign3 = 52;
int ign4 = 50;
#endif
  int tipo_ignicao = 1;//1 roda fonica e 2 distribuidor
  int qtd_dente = 12; //60 
  int qtd_dente_faltante = 1; //2
  int local_rodafonica = 2; // 2 para virabrequinho e 1 para comando
  int qtd_cilindro = 6 / local_rodafonica;
  int grau_pms = 60;
  int dwell_bobina = 4;
  int dwell_partida = 5;
  int dwell_funcionamento = 3;

int tipo_ignicao_sequencial = 0;// sequencial 1 semi-sequencial 0
volatile unsigned int qtd_voltas = 0;
int grau_cada_dente = 360 / qtd_dente;
int grau_avanco = 0;
int grau_avanco_partida = 1; // avanço definido apenas na partida
int grau_entre_cada_cilindro = 360 / qtd_cilindro;
int posicao_atual_sensor = 0;
volatile unsigned int leitura = 0;
int qtd_leitura = 0;
int referencia_leitura = 1;//map 1 e tps 2
int modo_ignicao = 1; // 1 para centelha perdida e 2 para centelha unica  
int avanco_fixo = 0; // avanço fixo 0 desligado e 1 ligado
int grau_avanco_fixo = 0; // grau de avanço fixo de 0 a 360 mais usado para calibrar o pms
int tipo_sinal_bobina = 1 ; // 1 alto e 0 baixo tipo de sinal enviado para bobina ente alto ou baixo conforme modelo da bobina

volatile unsigned long tempo_anterior = 0;
volatile unsigned long tempo_dente_anterior[2] = {0,0};
volatile unsigned long tempo_dente_anterior_pms;
volatile unsigned long tempo_inicio_volta_completa = 0;
volatile unsigned long tempo_final_volta_completa = 0;
volatile unsigned long tempo_total_volta_completa = 0;
volatile unsigned long tempo_cada_grau = 0;
volatile unsigned long tempo_proxima_ignicao[8];
volatile unsigned long tempo_atual = 0;
volatile unsigned long tempo_atual_proxima_ignicao[8];
//volatile unsigned long tempo_inicio_dwell;
//volatile unsigned long tempo_final_dwell;
volatile unsigned long intervalo_tempo_entre_dente = 0;
volatile unsigned long verifica_falha = 0;
int inicia = 1;
volatile long falha = 0;
int qtd_revolucoes = 0;
int qtd_perda_sincronia = 0;
//volatile int cilindro = 0;
volatile int cilindro_anterior = -1;
int cilindro_ign = 0;
int qtd_loop = 0;
unsigned long intervalo_execucao = 200; // intervalo em milissegundos
unsigned long ultima_execucao = 0;       // variável para armazenar o tempo da última execução
unsigned long tempo_inicial_rpm; // Variáveis para registrar o tempo inicial do rpm
unsigned long tempo_final_rpm;  // Variáveis para registrar o tempo final do rpm
//volatile unsigned long ti = 0;
//volatile unsigned long tf;
volatile unsigned int rpm = 0;
volatile int rpm_anterior = 0;
int rpm_partida = 400;
const int ignicao_pins[] = {ign1, ign2, ign3, ign4, ign1, ign2, ign3, ign4}; // Array com os pinos de ignição

// Declare as variáveis para controlar o estado do pino de saída
volatile bool captura_dwell[8] = {false, false, false, false, false, false, false, false};
volatile bool ign_acionado[8] = {false, false, false, false, false, false, false, false};
volatile unsigned long tempo_percorrido[8];
volatile bool flag_interrupcao = false;
// variaveis reverente a entrada de dados pela serial
const int MAX_VALUES = 270; // tamanho máximo do vetor
int values[MAX_VALUES];     // vetor para armazenar os valores recebidos
int matrix[16][16];
int vetor_map[16];
int vetor_rpm[16];
int index = 0;   // índice atual do vetor
int indice_envio = 0;   // índice atual do vetor de envio
char buffer[6]; // buffer temporário para armazenar caracteres recebidos
int tipo_vetor_map = 0;
int tipo_vetor_rpm = 0;
int tipo_vetor_matrix = 0;
int tipo_vetor_configuracao_inicial = 0;
int tipo_vetor_configuracao_faisca = 0;
int tipo_vetor_configuracao_dwell = 0;
int tipo_vetor_configuracao_clt = 0;
bool status_dados_tempo_real = false;
volatile int valor_map = 10;
int ajuste_pms =  0;
int busca_avanco_linear = true;
int referencia_temperatura_clt1 = 20;
int referencia_resistencia_clt1 = 2500;
int referencia_temperatura_clt2 = 100;
int referencia_resistencia_clt2 = 187;
int temperatura_motor = 0;

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

int ler_valor_eeprom_2byte(int endereco) {
    // Lê os dois bytes da EEPROM
    byte byte_menos_significativo = EEPROM.read(endereco);
    byte byte_mais_significativo = EEPROM.read(endereco + 1);

    // Reconstrói o valor inteiro a partir dos bytes lidos
    int valor = byte_mais_significativo << 8 | byte_menos_significativo;

    return valor;
}
void gravar_dados_eeprom_tabela_ignicao_map_rpm() {
  Serial.println("Gravando dados");
  int highByte;
  int lowByte;
  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
  for (int i = 0; i < 16; i++) {
    highByte = vetor_rpm[i] >> 8; // Obtém o byte mais significativo
    lowByte = vetor_rpm[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.write(i * 2 + 10, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.write(i * 2 + 11, lowByte); // Armazena o byte menos significativo na posição i*2+11
  }
  // Escrita da matrix na EEPROM
  for (int i = 0; i < 16; i++) {
    EEPROM.write(i + 50, vetor_map[i]); // Endereço de memória começa em 50
    for (int j = 0; j < 16; j++) {
      EEPROM.write(100 + i * 16 + j, matrix[i][j]); // Endereço de memória começa em 100, fim em 255 para a matriz
    }
  }
}

void verificar_dados_eeprom_tabela_ignicao_map_rpm() {
  Serial.println("Verificando dados gravado da tabela");
  // Verificar dados do vetor_rpm na EEPROM
  for (int i = 0; i < 16; i++) {
    int storedValue;
    int storedHighByte = EEPROM.read(i * 2 + 10);
    int storedLowByte = EEPROM.read(i * 2 + 11);
    storedValue = (storedHighByte << 8) | storedLowByte;
    Serial.print("Gravação na posição ");
      Serial.print(i);
      Serial.print(": valor lido da EEPROM = ");
      Serial.print(storedValue);
      Serial.print(", valor esperado = ");
      Serial.println(vetor_rpm[i]);
    if (storedValue != vetor_rpm[i]) {
      Serial.print("Erro de gravação na posição ");
      Serial.print(i);
      Serial.print(": valor lido da EEPROM = ");
      Serial.print(storedValue);
      Serial.print(", valor esperado = ");
      Serial.println(vetor_rpm[i]);
    }
  }

  // Verificar dados do vetor_map e da matrix na EEPROM
  for (int i = 0; i < 16; i++) {
    int storedValue = EEPROM.read(i + 50);
    if (storedValue != vetor_map[i]) {
      Serial.print("Erro de gravação na posição ");
      Serial.print(i);
      Serial.print(" do vetor_map: valor lido da EEPROM = ");
      Serial.print(storedValue);
      Serial.print(", valor esperado = ");
      Serial.println(vetor_map[i]);
    }
    for (int j = 0; j < 16; j++) {
      storedValue = EEPROM.read(100 + i * 16 + j);
      if (storedValue != matrix[i][j]) {
        Serial.print("Erro de gravação na posição ");
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(" da matriz: valor lido da EEPROM = ");
        Serial.print(storedValue);
        Serial.print(", valor esperado = ");
        Serial.println(matrix[i][j]);
      }
    }
  }
}
void gravar_dados_eeprom_configuracao_inicial() {
    int16_t grau_pms_16bit = (int16_t)grau_pms + 360; // Adiciona um offset de 360 para garantir que o valor seja sempre positivo
    if (grau_pms_16bit < 0) { // Verifica se o valor é negativo
        grau_pms_16bit = ~grau_pms_16bit + 1; // Calcula o valor em complemento de dois
    }
    uint8_t highByte = (uint8_t)(grau_pms_16bit >> 8); // Obtém o byte mais significativo
    uint8_t lowByte = (uint8_t)(grau_pms_16bit & 0xFF); // Obtém o byte menos significativo
    EEPROM.write(0*2+360, tipo_ignicao); // Armazena o byte na posição i*2+260
    EEPROM.write(1*2+360, qtd_dente);
    EEPROM.write(2*2+360, local_rodafonica);
    EEPROM.write(3*2+360, qtd_dente_faltante);
    EEPROM.write(4*2+360, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.write(4*2+361, lowByte); // Armazena o byte menos significativo na posição i*2+11
    EEPROM.write(5*2+360, qtd_cilindro);
}

void gravar_dados_eeprom_configuracao_faisca() {
    EEPROM.write(1*2+380, referencia_leitura);
    EEPROM.write(2*2+380, modo_ignicao);
    EEPROM.write(3*2+380, grau_avanco_partida);
    EEPROM.write(4*2+380, avanco_fixo);
    EEPROM.write(5*2+380, grau_avanco_fixo);
    EEPROM.write(6*2+380, tipo_sinal_bobina); 
}

void gravar_dados_eeprom_configuracao_dwell() {
    EEPROM.write(1*2+400, dwell_partida);
    EEPROM.write(2*2+400, dwell_funcionamento); 
}

void gravar_dados_eeprom_configuracao_clt() {
    // Gravar os valores divididos em bytes
    EEPROM.write(410, referencia_temperatura_clt1 & 0xFF);        // Byte menos significativo
    //EEPROM.write(411, (referencia_temperatura_clt1 >> 8) & 0xFF);  // Byte mais significativo
    EEPROM.write(412, referencia_resistencia_clt1 & 0xFF);
    EEPROM.write(413, (referencia_resistencia_clt1 >> 8) & 0xFF);
    EEPROM.write(414, referencia_temperatura_clt2 & 0xFF);
    //EEPROM.write(415, (referencia_temperatura_clt2 >> 8) & 0xFF);
    EEPROM.write(416, referencia_resistencia_clt2 & 0xFF);
    EEPROM.write(417, (referencia_resistencia_clt2 >> 8) & 0xFF);
}

void ler_dados_eeprom(){
  // Leitura dos valores RPM da EEPROM
  int highByte;
  int lowByte;
for (int i = 0; i < 16; i++) {
    highByte = EEPROM.read(i*2+10); // Lê o byte mais significativo da posição i*2+10
    lowByte = EEPROM.read(i*2+11); // Lê o byte menos significativo da posição i*2+11
    vetor_rpm[i] = (highByte << 8) | lowByte; // Recria o valor original a partir dos dois bytes lidos  
}
// Leitura map e matrix ponto da EEPROM
for (int i = 0; i < 16; i++) {
  vetor_map[i] = EEPROM.read(i+50);
  for (int j = 0; j < 16; j++) {
    matrix[i][j] = EEPROM.read(100 + i*16 + j);
  }
}


for (int i = 0; i < 16; i++) {
  int storedValue = EEPROM.read(i+50);
  if (storedValue != vetor_map[i]) {
    Serial.print("Erro de gravação na posição ");
    Serial.print(i);
    Serial.print(" do vetor_map: valor lido da EEPROM = ");
    Serial.print(storedValue);
    Serial.print(", valor esperado = ");
    Serial.println(vetor_map[i]);
  }
  for (int j = 0; j < 16; j++) {
    storedValue = EEPROM.read(100 + i*16 + j);
    if (storedValue != matrix[i][j]) {
      Serial.print("Erro de gravação na posição ");
      Serial.print(i);
      Serial.print(",");
      Serial.print(j);
      Serial.print(" da matriz: valor lido da EEPROM = ");
      Serial.print(storedValue);
      Serial.print(", valor esperado = ");
      Serial.println(matrix[i][j]);
    }
  }
}
//leitura dos dados de configurações iniciais
tipo_ignicao = EEPROM.read(0*2+360); 
qtd_dente = EEPROM.read(1*2+360);
local_rodafonica = EEPROM.read(2*2+360);
qtd_dente_faltante = EEPROM.read(3*2+360);
highByte = EEPROM.read(4*2+360); // Lê o byte mais significativo 
lowByte = EEPROM.read(4*2+360+1); // Lê o byte menos significativo
grau_pms = (highByte << 8) | lowByte; 
grau_pms = grau_pms - 360; //volta os dados para valor original 
qtd_cilindro = EEPROM.read(5*2+360);
grau_entre_cada_cilindro = 360 / qtd_cilindro;
grau_cada_dente = 360 / qtd_dente;

//leitura dos dados de configurações de faisca 
referencia_leitura = EEPROM.read(1*2+380); 
modo_ignicao = EEPROM.read(2*2+380);
grau_avanco_partida = EEPROM.read(3*2+380);
avanco_fixo = EEPROM.read(4*2+380);
grau_avanco_fixo = EEPROM.read(5*2+380);
tipo_sinal_bobina = EEPROM.read(6*2+380);

//leitura dos dados de configurações de dwell 
dwell_partida = EEPROM.read(1*2+400); 
dwell_funcionamento = EEPROM.read(2*2+400);

//leitura dos dados de calibração de ctl
referencia_temperatura_clt1 = EEPROM.read(410);
referencia_resistencia_clt1 = ler_valor_eeprom_2byte(412);
referencia_temperatura_clt2 = EEPROM.read(414);
referencia_resistencia_clt2 = ler_valor_eeprom_2byte(416); 

Serial.print("a,");
      // vetor map
      for (int  i = 0; i < 16; i++)
      {
        Serial.print(vetor_map[i]);
        Serial.print(",");
      }
      Serial.print(";");

      Serial.print("b,");
      // vetor rpm
      for (int  i = 0; i < 16; i++)
      {
        Serial.print(vetor_rpm[i]);
        Serial.print(",");
      }
      Serial.print(";");
      //matrix ponto 
      // transforma matriz em vetor
      Serial.print("c,");
        int k = 0;
        for (int i = 0; i < 16; i++)
        {
          for (int j = 0; j < 16; j++)
          {
            Serial.print(matrix[i][j]);
            Serial.print(",");
            k++;
          }
        }
        Serial.print(";");

      // dados configuração inicial
      Serial.print("g,");
      Serial.print(tipo_ignicao);
      Serial.print(",");
      Serial.print(qtd_dente);
      Serial.print(",");
      Serial.print(local_rodafonica);
      Serial.print(",");
      Serial.print(qtd_dente_faltante);
      Serial.print(",");
      Serial.print(grau_pms);
      Serial.print(",");
      Serial.print(qtd_cilindro * local_rodafonica);
      Serial.print(",");
      Serial.print(";");

      // dados configuração faisca
      Serial.print("j,");
      Serial.print(referencia_leitura);
      Serial.print(",");
      Serial.print(modo_ignicao);
      Serial.print(",");
      Serial.print(grau_avanco_partida);
      Serial.print(",");
      Serial.print(avanco_fixo);
      Serial.print(",");
      Serial.print(grau_avanco_fixo);
      Serial.print(",");
      Serial.print(tipo_sinal_bobina);
      Serial.print(",");
      Serial.print(";");

      // dados configuração dwell
      Serial.print("k,");
      Serial.print(dwell_partida);
      Serial.print(",");
      Serial.print(dwell_funcionamento);
      Serial.print(",");
      Serial.print(";");

      // dados configuração calibrate temperature sensor ctl
      Serial.print("l,");//letra L minusculo
      Serial.print(referencia_temperatura_clt1);
      Serial.print(",");
      Serial.print(referencia_resistencia_clt1);
      Serial.print(",");
      Serial.print(referencia_temperatura_clt2);
      Serial.print(",");
      Serial.print(referencia_resistencia_clt2);
      Serial.print(",");
      Serial.print(";");
}
int procura_indice(int value, int *arr, int size)
{
  int index = 0;
  int closest = abs(arr[0] - value);
  for (int i = 1; i < size; i++)
  {
    int diff = abs(arr[i] - value);
    if (diff < closest)
    {
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

float temperatura_clt(){
  int sensor_clt = analogRead(pino_sensor_clt);  // Lê o valor analógico
  float voltage_clt = sensor_clt * (5.0 / 1023.0);     // Converte o valor para tensão (0 a 5V)
  //float resistencia = 2000.0 * (5.0 / voltage_clt - 1.0);  // Resistência usando um resistor conhecido em ohms
  float resistencia_total = 2400.0;  // Substitua pelo valor real do seu uso
  float resistencia = resistencia_total * sensor_clt / 1023.0;
  //rpm = resistencia;
  float beta = calculateBeta(referencia_resistencia_clt1, referencia_temperatura_clt1, referencia_resistencia_clt2, referencia_temperatura_clt2);  
  //return calculateTemperature(resistencia, beta, referencia_resistencia_clt1, referencia_temperatura_clt1);
  int resultado_temperatura = calculateTemperature(resistencia, beta, referencia_resistencia_clt1, referencia_temperatura_clt1);

  if (resultado_temperatura > 250 || resultado_temperatura < 0) {
      return 250;
  } else {
      return resultado_temperatura;
  }
}
void envia_dados_tempo_real(int indice_envio){
    if (status_dados_tempo_real){
      if(indice_envio == 1){
        enviar_byte_serial(rpm_anterior, 2);
        enviar_byte_serial(valor_map, 1);
        enviar_byte_serial(temperatura_motor, 1);
        enviar_byte_serial(grau_avanco, 1);
        enviar_byte_serial(qtd_loop*5, 2);
        enviar_byte_serial(qtd_perda_sincronia, 1);
      }
    } 
}
void protege_ignicao(){
  if(rpm_anterior < 20){
    digitalWrite(ignicao_pins[0],0);
    digitalWrite(ignicao_pins[1],0);
    digitalWrite(ignicao_pins[2],0);
    digitalWrite(ignicao_pins[3],0);
  }
}
void leitura_entrada_dados_serial()
{
  if (Serial.available() > 0)
  {
    char data = Serial.read();
    if (data == 'a')//entrada de dados do vetor map
    {
      tipo_vetor_map = 1;
    }
    if (data == 'b')//entrada de dados do vetor rpm
    {
      tipo_vetor_rpm = 1;//b,500,600,700,800,900,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000,11000; exmplo
    }
    if (data == 'c')//entrada de dados do da matrix em vetor
    {
      tipo_vetor_matrix = 1;
    }
    if (data == 'd')// livre para reutilizar 
    {
      //função aqui
    }
    if (data == 'e')//retorna dados da tabela caso e
    { 
      // Leitura dos valores da EEPROM
      ler_dados_eeprom();
    }
     if (data == 'f'){//grava dados na eeprom
      // Escrita dos valores na EEPROM
      gravar_dados_eeprom_tabela_ignicao_map_rpm();
      verificar_dados_eeprom_tabela_ignicao_map_rpm();
      //gravar_dados_eeprom_configuracao_inicial();
      
    }
    if (data == 'g'){//grava configuração inicial
      tipo_vetor_configuracao_inicial = 1;
    }
     if (data == 'h')//retorna dados da ecu
    { 
      // Leitura dos valores da EEPROM
      ler_dados_eeprom();
      //delay(2000);
    }
    if (data == 'i') {
     if(status_dados_tempo_real){
      status_dados_tempo_real = false;
     }else{
      status_dados_tempo_real = true;
     }
    }
    if (data == 'j') {// configuração da faisca
      tipo_vetor_configuracao_faisca = 1;
    }
    if (data == 'k') {// configuração da faisca
      tipo_vetor_configuracao_dwell = 1;
    }
    if (data == 'l') {// configuração sensor temperatura clt
      tipo_vetor_configuracao_clt = 1;
    }

    if (data == ';')
    { // final do vetor
      
      if (tipo_vetor_map)
      {
        for (int i = 0; i < 16; i++)
        {
          vetor_map[i] = values[i];
        }
        tipo_vetor_map = 0;
        
      }
      if (tipo_vetor_rpm)
      {
        for (int i = 0; i < 16; i++)
        {
          vetor_rpm[i] = values[i];
        }
        tipo_vetor_rpm = 0;
      
      }
      if (tipo_vetor_matrix)
      {
        // transforma o vetor em matriz
        int k = 0;
        for (int i = 0; i < 16; i++)
        {
          for (int j = 0; j < 16; j++)
          {
            matrix[i][j] = values[k];
            k++;
          }
        }
        tipo_vetor_matrix = 0;
      }
      if (tipo_vetor_configuracao_inicial == 1){
          tipo_ignicao = values[0];
          qtd_dente = values[1];
          local_rodafonica = values[2]; // 2 para virabrequinho e 1 para comando
          qtd_dente_faltante = values[3];
          grau_pms = values[4];
          qtd_cilindro = values[5] / local_rodafonica;
          grau_entre_cada_cilindro = 360 / qtd_cilindro;
          gravar_dados_eeprom_configuracao_inicial();
          tipo_vetor_configuracao_inicial = 0;
      }
      if (tipo_vetor_configuracao_faisca == 1){
          referencia_leitura = values[0];//map 1 e tps 2
          modo_ignicao = values[1]; // 1 para centelha perdida e 2 para centelha unica
          grau_avanco_partida = values[2]; // avanço definido apenas na partida 
          avanco_fixo = values[3]; // avanço fixo 0 desligado e 1 ligado
          grau_avanco_fixo = values[4]; // grau de avanço fixo de 0 a 360 mais usado para calibrar o pms
          tipo_sinal_bobina = values[5] ; // 1 alto e 0 baixo tipo de sinal enviado para bobina ente alto ou baixo conforme modelo da bobina
          gravar_dados_eeprom_configuracao_faisca();
          tipo_vetor_configuracao_faisca = 0;
      }
      if (tipo_vetor_configuracao_dwell == 1){
          dwell_partida = values[0];
          dwell_funcionamento = values[1];
          gravar_dados_eeprom_configuracao_dwell();
          tipo_vetor_configuracao_dwell = 0;
      }
      if (tipo_vetor_configuracao_clt == 1){
          referencia_temperatura_clt1 = values[0];
          referencia_resistencia_clt1 = values[1];
          referencia_temperatura_clt2 = values[2];
          referencia_resistencia_clt2 = values[3];
          gravar_dados_eeprom_configuracao_clt();
          tipo_vetor_configuracao_clt = 0;
      }

      index = 0; // reinicia índice do vetor
    }
    else if (isdigit(data))
    {                                
      // Valor do vetor
      //Serial.print("Caractere recebido: ");
      //Serial.println(data);
      buffer[strlen(buffer)] = data; // adiciona o caractere recebido no buffer temporário
      if (strlen(buffer) >= sizeof(buffer) - 1)
      {                                    // verifica se o buffer está cheio
        buffer[sizeof(buffer) - 1] = '\0'; // adiciona um terminador de string para evitar um buffer overflow
      }
    }
    else if (data == ',' && strlen(buffer) > 0)
    {                                 
      //Serial.print("Número recebido como string: ");
      //Serial.println(buffer);
      buffer[strlen(buffer)] = '\0';  // adiciona um terminador de string para converter o buffer em uma string válida
      //values[index++] = atoi(buffer); // adiciona ao vetor
      //Substituído por strtol
      values[index++] = strtol(buffer, NULL, 10);
      if (index >= MAX_VALUES)
      {
        index = 0; // reinicia índice do vetor se estiver cheio
      }
      memset(buffer, 0, sizeof(buffer)); // reinicia o buffer
    }
  }
}

void leitor_sensor_roda_fonica()
{
  noInterrupts();
  qtd_leitura++;
  tempo_atual = micros() ;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);
  //verifica_falha = (tempo_dente_anterior[leitura] / 2) + tempo_dente_anterior[leitura];
  verifica_falha = (tempo_dente_anterior[leitura] / 2) + (tempo_dente_anterior[leitura] * qtd_dente_faltante);

  if (inicia){
    tempo_anterior = tempo_atual;
    tempo_dente_anterior[0] = tempo_anterior;
    inicia = 0;
  }
  if (leitura == 0){
    leitura = 1;
    tempo_dente_anterior[0] = (tempo_atual - tempo_anterior);
  }
  else{
    leitura = 0;
    tempo_dente_anterior[1] = (tempo_atual - tempo_anterior);
  }
  //Serial.print("|");
  //Serial.print(qtd_leitura); 
if (verifica_falha < intervalo_tempo_entre_dente && (intervalo_tempo_entre_dente < (tempo_dente_anterior[leitura] * (qtd_dente_faltante * 4))))
  {
    if (qtd_voltas == 1){
      tempo_final_volta_completa = tempo_atual;
      tempo_total_volta_completa = (tempo_final_volta_completa - tempo_inicio_volta_completa);
      qtd_voltas = 0;
    }
    if (qtd_voltas == 0){
      tempo_inicio_volta_completa = tempo_atual;
      qtd_voltas = 1;
    }
    //Serial.print("__");
    //Serial.println("");
    //Serial.print(posicao_atual_sensor); 
    tempo_final_rpm = micros();
    long delta = tempo_final_rpm - tempo_inicial_rpm;
    if(local_rodafonica == 1){
      rpm = (60) / (float(delta) / 1000000) * 2;
    }else{
      rpm = (60) / (float(delta) / 1000000);
    } 
    tempo_inicial_rpm = tempo_final_rpm;
    qtd_revolucoes++;
    tempo_cada_grau = tempo_total_volta_completa / 360;
    // posicao_atual_sensor = grau_cada_dente * qtd_dente_faltante;
    posicao_atual_sensor = 0;
   if ((qtd_leitura != (qtd_dente - qtd_dente_faltante))) {
    qtd_perda_sincronia++;
      if(qtd_perda_sincronia >=255){
        qtd_perda_sincronia = 0;
      }
    }else{
      falha++;
    }
    qtd_leitura = 0;
    falha++;// reservado para escapar rotação caso necessario no futuro   
    if(local_rodafonica == 1 && tipo_ignicao_sequencial == 0 ){  
    tempo_atual_proxima_ignicao[0] = tempo_atual;
    //cilindro = 1;
    ign_acionado[0] = false;
    captura_dwell[0] = false; 
    }
    if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
      tempo_atual_proxima_ignicao[0] = tempo_atual;
      ign_acionado[0] = false;
      captura_dwell[0] = false; 
    }
  }else{
    //tempo_cada_grau = intervalo_tempo_entre_dente / (360 / qtd_dente);
    //enviar_byte_serial(posicao_atual_sensor, 1);
  }
  posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;
  tempo_anterior = tempo_atual;
  interrupts();
}
void setup(){
  ler_dados_eeprom();//aqui le os dados da eeprom que forem salvo anteriormente
  delay(1000);  
  pinMode(ign1, OUTPUT);
  pinMode(ign2, OUTPUT);
  pinMode(ign3, OUTPUT);
  pinMode(ign4, OUTPUT);
  pinMode(pino_sensor_roda_fonica, INPUT_PULLUP);
  pinMode(pino_sensor_map, INPUT);
  attachInterrupt(digitalPinToInterrupt(pino_sensor_roda_fonica), leitor_sensor_roda_fonica, RISING);
  Serial.begin(9600);
}
void loop(){ 
    qtd_loop++;   
    if(rpm < rpm_partida){
      grau_avanco = grau_avanco_partida;
      dwell_bobina = dwell_partida;
    }
    else if(avanco_fixo){
      grau_avanco = grau_avanco_fixo;
      dwell_bobina = dwell_funcionamento;
    }
    else if(rpm < 3000 && busca_avanco_linear == true){
      int grau_minimo = matrix[procura_indice(valor_map, vetor_map, 16)][procura_indice(rpm, vetor_rpm, 16)];
      int indice_rpm_minimo = procura_indice(rpm, vetor_rpm, 16);
      int grau_maximo = matrix[procura_indice(valor_map, vetor_map, 16)][procura_indice(rpm, vetor_rpm, 16)+1];
      int grau_linear = busca_linear(rpm, vetor_rpm[indice_rpm_minimo], grau_minimo, vetor_rpm[indice_rpm_minimo+1], grau_maximo);
      grau_avanco = grau_linear;
      dwell_bobina = dwell_funcionamento;
    }
    else{
      grau_avanco = matrix[procura_indice(valor_map, vetor_map, 16)][procura_indice(rpm, vetor_rpm, 16)];
      dwell_bobina = dwell_funcionamento;
    }
    valor_map = map(analogRead(pino_sensor_map), 0, 1023, vetor_map[0], vetor_map[15]);
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
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000ul) >= tempo_proxima_ignicao[i]) && 
        (falha > 1)){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
  }
  for (int i = qtd_cilindro / 2; i < qtd_cilindro; i++){  
  tempo_proxima_ignicao[i] = (ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000ul) >= tempo_proxima_ignicao[i]) && 
        (falha > 1)){
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i - qtd_cilindro/2], 1);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;
    }
  }
  for (int i = 0; i < qtd_cilindro; i++) {
    if (captura_dwell[i] == true) {
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000ul)) {
            int verifica_posicao = ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i);
            if(((verifica_posicao >= posicao_atual_sensor - grau_cada_dente) && (rpm < 3000))){          
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
  }
}

if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0
  if (grau_pms <= 180) {
    if (grau_pms < 60 || rpm > 3000) {
        ajuste_pms = 180;
    }else{
      ajuste_pms = 0;
    } 
}
   for (int i = 0; i < qtd_cilindro; i++){
    tempo_proxima_ignicao[i] = ( ajuste_pms + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i) ) * tempo_cada_grau;
    // IGN
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + dwell_bobina * 1000ul >= tempo_proxima_ignicao[i]))
    {
        captura_dwell[i] = true;
        tempo_percorrido[i] = tempo_atual;
        digitalWrite(ignicao_pins[i], 1);
        tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
        ign_acionado[i] = true;
        ign_acionado[i+1] = false;
        captura_dwell[i+1] = false;        
    }
        }

    for (int i = 0; i < qtd_cilindro; i++) {
    if ((captura_dwell[i] == true) && (ign_acionado[i] == true)) {
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000ul)) {
            captura_dwell[i] = false;
            //ign_acionado[i] = false;
            digitalWrite(ignicao_pins[i], 0);
            //delay(5); //um pequeno atraso
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
  // Serial.println(qtd_perda_sincronia); 
   // atualiza o tempo da última execução
   ultima_execucao = millis();
   qtd_loop = 0;
  }
}
