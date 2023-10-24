#include <Arduino.h>
#include <EEPROM.h>


#define pino_sensor_roda_fonica 19
#define pino_sensor_fase 18
#define pino_sensor_map A3

  int tipo_ignicao = 1;//1 roda fonica e 2 distribuidor
  int qtd_dente = 12; //60 
  int qtd_dente_faltante = 1; //2
  int local_rodafonica = 1; // 2 para virabrequinho e 1 para comando
  int qtd_cilindro = 6 / local_rodafonica;
  int grau_pms = 30;
  int dwell_bobina = 3;

int ign1 = 40;
int ign2 = 38;
int ign3 = 52;
int ign4 = 50;

int tipo_ignicao_sequencial = 0;// sequencial 1 semi-sequencial 0
volatile unsigned int qtd_voltas = 0;
int grau_cada_dente = 360 / qtd_dente;
int grau_avanco = 0;
int grau_avanco_partida = 5;
int grau_entre_cada_cilindro = 360 / qtd_cilindro;
int posicao_atual_sensor = 0;
volatile unsigned int leitura = 0;
volatile unsigned int qtd_leitura = 0;

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
volatile unsigned long tempo_inicio_dwell;
volatile unsigned long tempo_final_dwell;
volatile unsigned long intervalo_tempo_entre_dente = 0;
volatile unsigned long verifica_falha = 0;
int inicia = 1;
volatile int pms = 0;
volatile long falha = 0;
volatile unsigned long qtd_revolucoes = 0;
volatile int cilindro = 0;
volatile int cilindro_anterior = -1;
int cilindro_ign = 0;
int dente = 0;
unsigned long intervalo_execucao = 1000; // intervalo de 1 segundo em milissegundos
unsigned long ultima_execucao = 0;       // variável para armazenar o tempo da última execução
volatile int pulsos;
unsigned long tempo_inicial_rpm; // Variáveis para registrar o tempo inicial do rpm
unsigned long tempo_final_rpm;  // Variáveis para registrar o tempo final do rpm
//volatile unsigned long ti = 0;
//volatile unsigned long tf;
volatile unsigned int rpm = 0;
volatile int rpm_anterior = 0;
const int ignicao_pins[] = {ign1, ign2, ign3, ign4, ign1, ign2, ign3, ign4}; // Array com os pinos de ignição

// Declare as variáveis para controlar o estado do pino de saída
volatile bool captura_dwell[8] = {false, false, false, false, false, false, false, false};
volatile bool ign_acionado[8] = {false, false, false, false, false, false, false, false};
volatile unsigned long tempo_percorrido[8];
// variaveis reverente a entrada de dados pela serial
const int MAX_VALUES = 500; // tamanho máximo do vetor
int values[MAX_VALUES];     // vetor para armazenar os valores recebidos
int matrix[16][16];
int vetor_map[16];
int vetor_rpm[16];
int index = 0;   // índice atual do vetor
char buffer[10]; // buffer temporário para armazenar caracteres recebidos
int tipo_vetor_map = 0;
int tipo_vetor_rpm = 0;
int tipo_vetor_matrix = 0;
int tipo_vetor_configuracao_inicial = 0;
bool status_dados_ponto_ignicao = false;
bool status_dados_tempo_real = false;
volatile int valor_map = 10;

void gravar_dados_eeprom_tabela_ignicao_map_rpm(){
  Serial.println("Gravando dados");
  int highByte;
  int lowByte;
  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
  //a primeira posição de escrita será a posição 10 ou 11 
  //(dependendo do valor do primeiro elemento do vetor_rpm) e a última posição de escrita será a posição 42.
for (int i = 0; i < 16; i++) {
    highByte = vetor_rpm[i] >> 8; // Obtém o byte mais significativo
    lowByte = vetor_rpm[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.write(i*2+10, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.write(i*2+11, lowByte); // Armazena o byte menos significativo na posição i*2+11
}
 // Escrita da matrix na EEPROM
 //A primeira chamada à função "EEPROM.write()" escreve 1 byte no endereço 50 da memória EEPROM, e cada iteração subsequente do loop escreve mais 1 byte no endereço seguinte. Assim, ao final do loop, a última posição escrita será a posição 65 (ou seja, 50 + 16).
//A segunda chamada à função "EEPROM.write()" escreve 1 byte no endereço 100 da memória EEPROM, e cada iteração subsequente do loop escreve mais 1 byte no endereço seguinte. Assim, ao final da execução do loop interno (j), a última posição escrita será a posição 255 (ou seja, 100 + 16*16 - 1).
//Portanto, a última posição da memória EEPROM utilizada pelo loop será a posição 255.
for (int i = 0; i < 16; i++) {
  EEPROM.write(i+50, vetor_map[i]);// endereço de memória começa em 50
  for (int j = 0; j < 16; j++) {
    EEPROM.write(100 + i*16 + j, matrix[i][j]); // endereço de memória começa em 100 fim 255 para a matriz
  }
}
for (int i = 0; i < 16; i++) {
    int storedValue;
    int storedHighByte = EEPROM.read(i*2+10);
    int storedLowByte = EEPROM.read(i*2+11);
    storedValue = (storedHighByte << 8) | storedLowByte;
    if (storedValue != vetor_rpm[i]) {
      Serial.print("Erro de gravação na posição ");
      Serial.print(i);
      Serial.print(": valor lido da EEPROM = ");
      Serial.print(storedValue);
      Serial.print(", valor esperado = ");
      Serial.println(vetor_rpm[i]);
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
Serial.println("Gravação finalizada");
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
void envia_dados_ponto_ignicao(){
    while(status_dados_ponto_ignicao)
    {
    // procura valor do rpm mais proximo e map para achar o ponto na matriz
      Serial.print(",");
      Serial.print("d,");
      Serial.print(procura_indice(map(analogRead(pino_sensor_map), 0, 1023, vetor_map[15], vetor_map[0]), vetor_map, 16));
      Serial.print(",");
      Serial.print(procura_indice(rpm, vetor_rpm, 16));
      Serial.print(",");
      Serial.print(rpm);
      Serial.print(",");
      Serial.print(matrix[procura_indice(map(analogRead(pino_sensor_map), 0, 1023, vetor_map[15], vetor_map[0]), vetor_map, 16)][procura_indice(rpm, vetor_rpm, 16)]);
      //Serial.print(",; ");
      delay(100);
      
    }
    
}
void envia_dados_tempo_real(){
    while (status_dados_tempo_real)
    {
      Serial.print(",");
      Serial.print(1);
      Serial.print(",");
      Serial.print(rpm);
      Serial.print(",");
      Serial.print(map(analogRead(pino_sensor_map), 0, 1023, vetor_map[15], vetor_map[0]));
      Serial.print(",");
      Serial.print(80);
      Serial.print(",");
      Serial.print(grau_avanco);
      //Serial.print(",");
      //Serial.print(";");
      delay(100);
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
      tipo_vetor_rpm = 1;
    }
    if (data == 'c')//entrada de dados do da matrix em vetor
    {
      tipo_vetor_matrix = 1;
    }
    if (data == 'd')// retorna ponto de igniçao 
    {
     if(status_dados_ponto_ignicao){
      status_dados_ponto_ignicao = false;
     }else{
      status_dados_ponto_ignicao = true;
      envia_dados_ponto_ignicao();
     }
    }
    if (data == 'e')//retorna dados da tabela caso e
    { 
      // Leitura dos valores da EEPROM
      ler_dados_eeprom();
    }
     if (data == 'f'){//grava dados na eeprom
      // Escrita dos valores na EEPROM
      gravar_dados_eeprom_tabela_ignicao_map_rpm();
      gravar_dados_eeprom_configuracao_inicial();
      
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
    if (data == 'i') {// retorna ponto de igniçao
     if(status_dados_tempo_real){
      status_dados_tempo_real = false;
     }else{
      status_dados_tempo_real = true;
      envia_dados_tempo_real();
     }
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
          
          tipo_vetor_configuracao_inicial = 0;
      }

      index = 0; // reinicia índice do vetor
    }
    else if (isdigit(data))
    {                                // valor do vetor
      buffer[strlen(buffer)] = data; // adiciona o caractere recebido no buffer temporário
      if (strlen(buffer) >= sizeof(buffer) - 1)
      {                                    // verifica se o buffer está cheio
        buffer[sizeof(buffer) - 1] = '\0'; // adiciona um terminador de string para evitar um buffer overflow
      }
    }
    else if (data == ',' && strlen(buffer) > 0)
    {                                 // final do número
      buffer[strlen(buffer)] = '\0';  // adiciona um terminador de string para converter o buffer em uma string válida
      values[index++] = atoi(buffer); // adiciona ao vetor
      if (index >= MAX_VALUES)
      {
        index = 0; // reinicia índice do vetor se estiver cheio
      }
      memset(buffer, 0, sizeof(buffer)); // reinicia o buffer
    }
  }
}

void inicializar_valores() {
     // seta os valores no vetor_map
      vetor_map[0] = 100;
      vetor_map[1] = 96;
      vetor_map[2] = 88;
      vetor_map[3] = 80;
      vetor_map[4] = 74;
      vetor_map[5] = 66;
      vetor_map[6] = 56;
      vetor_map[7] = 50;
      vetor_map[8] = 46;
      vetor_map[9] = 40;
      vetor_map[10] = 36;
      vetor_map[11] = 30;
      vetor_map[12] = 26;
      vetor_map[13] = 20;
      vetor_map[14] = 16;
      vetor_map[15] = 10;

      // seta os valores no vetor_rpm
      vetor_rpm[0] = 510;
      vetor_rpm[1] = 700;
      vetor_rpm[2] = 1200;
      vetor_rpm[3] = 1700;
      vetor_rpm[4] = 2200;
      vetor_rpm[5] = 2700;
      vetor_rpm[6] = 3200;
      vetor_rpm[7] = 3700;
      vetor_rpm[8] = 4200;
      vetor_rpm[9] = 4700;
      vetor_rpm[10] = 5200;
      vetor_rpm[11] = 5700;
      vetor_rpm[12] = 6200;
      vetor_rpm[13] = 6700;
      vetor_rpm[14] = 7200;
      vetor_rpm[15] = 7700;

    int matrix_padrao[16][16] = {
      {17,18,19,20,21,24,25,27,28,29,30,31,32,32,33,34},
      {17,19,19,20,21,24,25,27,28,29,30,31,32,32,33,34},
      {17,18,19,20,21,24,25,27,28,29,30,31,32,32,33,34},
      {17,20,21,20,21,24,25,27,28,28,30,31,31,31,32,33},
      {17,22,22,20,21,24,25,26,28,28,30,31,31,31,32,33},
      {17,20,20,20,21,24,25,26,27,28,29,30,30,30,31,32},
      {17,20,20,20,21,23,23,24,25,26,27,28,29,29,30,31},
      {17,20,20,20,21,22,22,23,24,25,26,27,28,28,29,30},
      {18,18,18,18,21,21,21,21,21,21,21,20,19,20,20,20},
      {18,18,18,18,20,20,20,20,19,19,19,18,18,18,18,18},
      {18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18},
      {17,17,17,16,16,16,16,16,18,18,18,18,17,17,17,17},
      {16,16,16,15,15,15,15,15,18,18,18,18,16,16,16,16},
      {15,15,15,14,14,14,14,14,18,18,18,16,13,13,13,13},
      {12,12,12,11,11,11,11,11,16,16,16,14,11,11,11,11},
      {11,11,11,9,9,9,9,9,15,15,15,13,10,10,10,17}
    };
    // atualização dos valores da matriz
    for(int i = 0; i < 16; i++) {
      for(int j = 0; j < 16; j++) {
        matrix[i][j] = matrix_padrao[i][j];
      }
    }
  
}


void leitor_sensor_roda_fonica()
{
  noInterrupts();
  qtd_leitura++;
  pulsos++;
  tempo_atual = micros() ;
  intervalo_tempo_entre_dente = (tempo_atual - tempo_anterior);
  verifica_falha = (tempo_dente_anterior[leitura] / 2) + tempo_dente_anterior[leitura];

  if (inicia)
  {
    tempo_anterior = tempo_atual;
    tempo_dente_anterior[0] = tempo_anterior;
    inicia = 0;
  }

  if (leitura == 0)
  {
    leitura = 1;
    tempo_dente_anterior[0] = (tempo_atual - tempo_anterior);
  }
  else
  {
    leitura = 0;
    tempo_dente_anterior[1] = (tempo_atual - tempo_anterior);
  }

  // Serial.print("|");
  if (verifica_falha < intervalo_tempo_entre_dente)
  {
    if (qtd_voltas == 1)
    {
      tempo_final_volta_completa = tempo_atual;
      tempo_total_volta_completa = (tempo_final_volta_completa - tempo_inicio_volta_completa);
      qtd_voltas = 0;
    }
    if (qtd_voltas == 0)
    {
      tempo_inicio_volta_completa = tempo_atual;
      qtd_voltas = 1;
    }

    // Serial.println("");
    // Serial.print("__");
    // tempo_final_rpm = micros();
    // long delta = tempo_final_rpm - tempo_inicial_rpm;
    // rpm = (60) / (float(delta) / 1000000);
    // tempo_inicial_rpm = tempo_final_rpm;
    qtd_revolucoes++;
    tempo_cada_grau = tempo_total_volta_completa / 360;

    posicao_atual_sensor = grau_cada_dente * qtd_dente_faltante;
    qtd_leitura = qtd_dente_faltante;
    falha++;
    pms = 1;
    
    if(rpm_anterior < 500){
      grau_avanco = grau_avanco_partida;
    }else{
      grau_avanco = matrix[procura_indice(valor_map, vetor_map, 16)][procura_indice(rpm_anterior, vetor_rpm, 16)];
    }
    
    if(tipo_ignicao_sequencial == 0 ){  
    tempo_atual_proxima_ignicao[0] = tempo_atual;
    cilindro = 1;
    ign_acionado[0] = false; 
    }
    // else{
    //   cilindro = 2;
    // tempo_atual_proxima_ignicao[0] = tempo_atual;
    //   //tempo_proxima_ignicao[0] = tempo_atual + (grau_pms * tempo_cada_grau);
    //   tempo_proxima_ignicao[0] = (grau_pms - grau_avanco + grau_entre_cada_cilindro) * tempo_cada_grau;
    //   //tempo_proxima_ignicao[1] = (grau_pms + (grau_entre_cada_cilindro * 2)) * tempo_cada_grau;
    //   //tempo_proxima_ignicao[2] = (grau_pms + (grau_entre_cada_cilindro * 3)) * tempo_cada_grau; 
    // }
    
  }
  posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;

  tempo_anterior = tempo_atual;
  interrupts();
}

void setup()
{
   // Chama a função para inicializar os valores da tabela de ponto grava e le caso nao use a interface 
  //inicializar_valores();
  //delay(1000);
  //aqui le os dados da eeprom que forem salvo anteriormente
  ler_dados_eeprom();
  //delay(1000);
  
  pinMode(ign1, OUTPUT);
  pinMode(ign2, OUTPUT);
  pinMode(ign3, OUTPUT);
  pinMode(ign4, OUTPUT);
  pinMode(pino_sensor_roda_fonica, INPUT_PULLUP);
  pinMode(pino_sensor_map, INPUT);
  attachInterrupt(digitalPinToInterrupt(pino_sensor_roda_fonica), leitor_sensor_roda_fonica, RISING);
  Serial.begin(9600);
}

void loop()
{ 
tempo_atual = micros() ;//salva sempre o tempo atual para verificaçoes

if(local_rodafonica == 1 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0

  for (int i = qtd_cilindro / 2; i <= qtd_cilindro; i++)
{
  tempo_proxima_ignicao[i] = (180 + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i)) * tempo_cada_grau;
    // IGN
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000) >= tempo_proxima_ignicao[i]) && 
        (falha > 3) && 
        (pms == 1) && 
        (rpm >= 100))
    {
      captura_dwell[i] = true;
      tempo_percorrido[i] = tempo_atual;
      digitalWrite(ignicao_pins[i - qtd_cilindro/2], 1);
      tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
      ign_acionado[i] = true;
      ign_acionado[i+1] = false;
      captura_dwell[i+1] = false; 
    }
}

  for (int i = 0; i <= qtd_cilindro/2; i++)
{
  tempo_proxima_ignicao[i] = (180 + grau_pms - grau_avanco + (grau_entre_cada_cilindro * i+1)) * tempo_cada_grau;
    // IGN
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + (dwell_bobina * 1000) >= tempo_proxima_ignicao[i]) && 
        (falha > 3) && 
        (pms == 1) && 
        (rpm >= 100))
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
    if (captura_dwell[i] == true) {
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000)) {
            captura_dwell[i] = false;
            if (i < qtd_cilindro/2) {
              digitalWrite(ignicao_pins[i], 0);
            }else{
              digitalWrite(ignicao_pins[i - qtd_cilindro/2], 0);
            }
            cilindro++;
           
        }
    }
  }
}

if(local_rodafonica == 2 && tipo_ignicao_sequencial == 0){ // 2 para virabrequinho e 1 para comando, sequencial 1 e semi 0

   for (int i = 0; i < qtd_cilindro; i++){
    tempo_proxima_ignicao[i] = (180+grau_pms - grau_avanco + (grau_entre_cada_cilindro * i+1) ) * tempo_cada_grau;
    // IGN
    if ((captura_dwell[i] == false) && (ign_acionado[i] == false) && 
        (tempo_atual - tempo_atual_proxima_ignicao[i] + dwell_bobina * 1000 >= tempo_proxima_ignicao[i]))
    {
      captura_dwell[i] = true;
      tempo_percorrido[i] = tempo_atual;
      digitalWrite(ignicao_pins[i], 1);
      tempo_atual_proxima_ignicao[i + 1] = tempo_atual_proxima_ignicao[i]; 
      ign_acionado[i] = true;
      ign_acionado[i+1] = false;
      //captura_dwell[i+1] = false;       
    }
        }

    for (int i = 0; i < qtd_cilindro; i++) {
    if (captura_dwell[i] == true) {
        if ((tempo_atual - tempo_percorrido[i]) >= (dwell_bobina * 1000)) {
            captura_dwell[i] = false;
            digitalWrite(ignicao_pins[i], 0);
        }
    }
  }
}

  valor_map = map(analogRead(pino_sensor_map), 0, 1023, vetor_map[15], vetor_map[0]);   
  leitura_entrada_dados_serial();
//   // verifica se já passou o intervalo de tempo
  if (millis() - ultima_execucao >= intervalo_execucao)
  {
  //gravar_dados_eeprom_configuracao_inicial();
  //gravar_dados_eeprom_tabela_ignicao_map_rpm();
  rpm = qtd_revolucoes  * 60 / (intervalo_execucao / 1000);
  rpm_anterior = rpm;
   
   //envia_dados_ponto_ignicao(valor_map, rpm_anterior);
  // envia_dados_tempo_real();
   protege_ignicao();
  
  //  Serial.print("Maior valor no vetor map:");
  //  Serial.println(vetor_map[0]);
  //  Serial.print("Menor valor no vetor map:");
  //  Serial.println(vetor_map[15]);
  //  Serial.print("Leitura no pino:");
  //  Serial.println(analogRead(pino_sensor_map));
  //  Serial.print("Valor convertido:");
  //  Serial.println(valor_map);
  //  Serial.print("indice do vetor:");
  //  Serial.println(procura_indice(valor_map, vetor_map, 16));
  //  Serial.print("valor do vetor:");
  //  Serial.println(vetor_map[procura_indice(valor_map, vetor_map, 16)]);
      
  // atualiza o tempo da última execução
   ultima_execucao = millis();

  }
}
