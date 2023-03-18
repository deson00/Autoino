#include <Arduino.h>
#include <EEPROM.h>

#define qtd_dente 60
#define qtd_dente_faltante 2
#define pino_sensor_roda_fonica 19
#define pino_sensor_fase 18

volatile int pulsos;
volatile unsigned long ti = 0;
volatile unsigned long tf;
volatile unsigned int rpm;
int inicia = 1;
int ign1 = 40;
int ign2 = 38;
int ign3 = 52;
int ign4 = 50;
volatile unsigned int dwell_bobina = 4;
volatile unsigned int local_rodafonica = 2; // 2 para virabrequinho e 1 para comando
volatile unsigned int qtd_cilindro = 6 / local_rodafonica;
volatile unsigned int qtd_voltas = 0;
volatile unsigned int grau_cada_dente = 360 / qtd_dente;
volatile unsigned int grau_avanco = 0;
volatile unsigned int grau_pms = 70;
volatile unsigned int grau_entre_cada_cilindro = 360 / qtd_cilindro;
volatile unsigned int posicao_atual_sensor = 0;
volatile unsigned int leitura = 0;
volatile unsigned int qtd_leitura = 0;
volatile unsigned int rpm_anterior = 0;
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
volatile int pms = 0;
volatile long falha = 0;
volatile int cilindro = 0;
volatile int cilindro_anterior = -1;
int cilindro_ign = 0;
int dente = 0;
volatile int tipo_ignicao_sequencial = 0;
unsigned long intervalo_execucao = 1000; // intervalo de 1 segundo em milissegundos
unsigned long ultima_execucao = 0;       // variável para armazenar o tempo da última execução

const int ignicao_pins[] = {ign1, ign2, ign3, ign4}; // Array com os pinos de ignição

// Declare as variáveis para controlar o estado do pino de saída
volatile int estado_pino_ignicao = LOW;  // variavel armazenha o estado do pino de ignição se esta ligado ou desligado
volatile bool estado_anterior_pino_ignicao = LOW;
volatile unsigned long tempo_inicio_pulso = 0;
const unsigned long duracao_pulso = 4; // duração do pulso em milissegundos
//volatile bool captura_dwell = false;
volatile bool captura_dwell[8] = {false, false, false, false, false, false, false, false};
volatile unsigned long tempo_percorrido[8];
volatile bool alternar_funcao = true;
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


void gravar_dados_eeprom(){
  // Escrita do vetor_rpm para valores de 2 bytes na EEPROM
for (int i = 0; i < 16; i++) {
  if (vetor_rpm[i] <= 255) {
    EEPROM.write(i*2+10, vetor_rpm[i]); // Armazena o byte na posição i*2+10
  } else {
    int highByte = vetor_rpm[i] >> 8; // Obtém o byte mais significativo
    int lowByte = vetor_rpm[i] & 0xFF; // Obtém o byte menos significativo
    EEPROM.write(i*2+10, highByte); // Armazena o byte mais significativo na posição i*2+10
    EEPROM.write(i*2+11, lowByte); // Armazena o byte menos significativo na posição i*2+11
  }
}

 // Escrita da matrix na EEPROM
for (int i = 0; i < 16; i++) {
  EEPROM.write(i+50, vetor_map[i]);
  for (int j = 0; j < 16; j++) {
    EEPROM.write(100 + i*16 + j, matrix[i][j]); // endereço de memória começa em 32 para a matriz
  }
}
}

void ler_dados_eeprom(){
  // Leitura dos valores da EEPROM
for (int i = 0; i < 16; i++) {
    int highByte = EEPROM.read(i*2+10); // Lê o byte mais significativo da posição i*2+10
    int lowByte = EEPROM.read(i*2+11); // Lê o byte menos significativo da posição i*2+11
    vetor_rpm[i] = (highByte << 8) | lowByte; // Recria o valor original a partir dos dois bytes lidos  
}

// Leitura vetor_map e matrix da EEPROM
for (int i = 0; i < 16; i++) {
  vetor_map[i] = EEPROM.read(i+50);
  for (int j = 0; j < 16; j++) {
    matrix[i][j] = EEPROM.read(100 + i*16 + j);
  }
}
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
  }
  return index;
}

void leitura_entrada_dados_serial()
{
  if (Serial.available() > 0)
  {
    char data = Serial.read();
    if (data == 'a')
    {
      tipo_vetor_map = 1;
    }
    if (data == 'b')
    {
      tipo_vetor_rpm = 1;
    }
    if (data == 'c')
    {
      tipo_vetor_matrix = 1;
    }
    if (data == 'd')
    {
      // procura valor do rpm mais proximo e map para achar o ponto na matriz
      Serial.print("d,");
      Serial.print(procura_indice(96, vetor_map, 16));
      Serial.print(",");
      Serial.print(procura_indice(rpm_anterior, vetor_rpm, 16));
      Serial.print(",");
      Serial.print(rpm_anterior);
      Serial.print(",;");
    }
    if (data == 'e')//retorna dados da tabela caso e
    { 
// Leitura dos valores da EEPROM
    ler_dados_eeprom();

delay(1000);
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
    }
     if (data == 'f'){
      // Escrita dos valores na EEPROM
      gravar_dados_eeprom();
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

void escrever_dados() {
  // Escreve os valores na EEPROM
  for (int i = 0; i < 16; i++) {
    EEPROM.write(i, vetor_map[i]);
  }
  for (int i = 16; i < 32; i++) {
    EEPROM.write(i, vetor_rpm[i - 16]);
  }
  for (int i = 32; i < 544; i += 2) {
    EEPROM.write(i, matrix[(i - 32) / 32][(i - 32) % 32] >> 8);
    EEPROM.write(i + 1, matrix[(i - 32) / 32][(i - 32) % 32] & 0xFF);
  }
}

void inicializar_valores() {
  // Lê os valores salvos na EEPROM
  for (int i = 0; i < 16; i++) {
    vetor_map[i] = EEPROM.read(i);
  }
  for (int i = 16; i < 32; i++) {
    vetor_rpm[i - 16] = EEPROM.read(i);
  }
  for (int i = 32; i < 544; i += 2) {
    matrix[(i - 32) / 32][(i - 32) % 32] = EEPROM.read(i) << 8 | EEPROM.read(i + 1);
  }

  // Verifica se os valores lidos são válidos
  bool dadosValidos = true;
  for (int i = 0; i < 16; i++) {
    if (vetor_map[i] < 0 || vetor_map[i] > 255) {
      dadosValidos = false;
      break;
    }
  }
  for (int i = 0; i < 16; i++) {
    if (vetor_rpm[i] < 0 || vetor_rpm[i] > 255) {
      dadosValidos = false;
      break;
    }
  }
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if (matrix[i][j] < 0 || matrix[i][j] > 65535) {
        dadosValidos = false;
        break;
      }
    }
    if (!dadosValidos) {
      break;
    }
  }

  // Se os dados lidos forem inválidos, seta os valores default
  if (!dadosValidos) {
    for (int i = 0; i < 16; i++) {
      vetor_map[i] = i;
      vetor_rpm[i] = i + 16;
      for (int j = 0; j < 16; j++) {
        matrix[i][j] = i * 16 + j;
      }
    }
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
  vetor_rpm[0] = 500;
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
  {11,11,11,9,9,9,9,9,15,15,15,13,10,10,10,10}
};
// atualização dos valores da matriz
for(int i = 0; i < 16; i++) {
  for(int j = 0; j < 16; j++) {
    matrix[i][j] = matrix_padrao[i][j];
  }
}
    // Salva os valores default na EEPROM
    escrever_dados();
  }
}


void leitor_sensor_roda_fonica()
{
  noInterrupts();
  qtd_leitura++;
  pulsos++;
  tf = micros();
  long delta = tf - ti;
  rpm = (60) / (float(delta) / 1000000 * 60);
  ti = tf;
  // calculo de mediana
  /*if(n % 2 == 0){
    md = n/2;
    mediana = (v[md] + v[md-1])/2;
  }
  else{
    md = n/2;
    mediana = v[md];
  })*/

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
    tempo_cada_grau = tempo_total_volta_completa / 360;

    posicao_atual_sensor = grau_cada_dente * qtd_dente_faltante;
    qtd_leitura = qtd_dente_faltante;
    falha++;
    pms = 1;
    
    //cilindro_ign = 0;
    grau_avanco = matrix[procura_indice(100, vetor_map, 16)][procura_indice(rpm_anterior, vetor_rpm, 16)];
    cilindro = 2;
    tempo_atual_proxima_ignicao[0] = tempo_atual;
      //tempo_proxima_ignicao[0] = tempo_atual + (grau_pms * tempo_cada_grau);
      tempo_proxima_ignicao[0] = (grau_pms - grau_avanco + grau_entre_cada_cilindro) * tempo_cada_grau;
      //tempo_proxima_ignicao[1] = (grau_pms + (grau_entre_cada_cilindro * 2)) * tempo_cada_grau;
      //tempo_proxima_ignicao[2] = (grau_pms + (grau_entre_cada_cilindro * 3)) * tempo_cada_grau;  

  }
  posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;

//IGN1
if ((captura_dwell[0] == false) && (tempo_proxima_ignicao[2] != 0) && (cilindro_ign == 4) && (tempo_atual - tempo_atual_proxima_ignicao[2] + (dwell_bobina * 1000) >= tempo_proxima_ignicao[2]) && (falha > 3) && (pms == 1) && (rpm >= 100))
{
  tempo_proxima_ignicao[2] = 0;
  cilindro_ign = 0 ;
  cilindro = 2;
  //Serial.print(cilindro_ign);
  //Serial.print(" x ");
  //Serial.println(tempo_proxima_ignicao[cilindro_ign]);
    
    //grau_avanco = matrix[procura_indice(100, vetor_map, 16)][procura_indice(rpm_anterior, vetor_rpm, 16)];
    //tempo_proxima_ignicao = tempo_atual + (tempo_cada_grau * grau_entre_cada_cilindro) + ((grau_pms * cilindro) * tempo_cada_grau);
    captura_dwell[0] = true;
    tempo_percorrido[0] = tempo_atual;
    digitalWrite(ignicao_pins[0],1);
    
  }

  if((tempo_atual - tempo_percorrido[0]) >= 4000){
    captura_dwell[0] = false;
    digitalWrite(ignicao_pins[0],0);
    
  }

//IGN2

if ((captura_dwell[1] == false) && (tempo_proxima_ignicao[0]) && (cilindro == 2) && (tempo_atual - tempo_atual_proxima_ignicao[0] + (dwell_bobina * 1000) >= tempo_proxima_ignicao[0]) && (falha > 3) && (pms == 1) && (rpm >= 100))
{
  tempo_proxima_ignicao[0] = 0;
    cilindro++;
    cilindro_ign = 3;
  tempo_atual_proxima_ignicao[1] = tempo_atual_proxima_ignicao[0];
  //tempo_proxima_ignicao[0] = (grau_pms + grau_entre_cada_cilindro) * tempo_cada_grau;
  tempo_proxima_ignicao[1] = (grau_pms - grau_avanco + (grau_entre_cada_cilindro * 2)) * tempo_cada_grau;
  //tempo_proxima_ignicao[2] = (grau_pms + (grau_entre_cada_cilindro * 3)) * tempo_cada_grau;  

  //Serial.print(cilindro_ign);
  //Serial.print(" x ");
  //Serial.println(tempo_proxima_ignicao[cilindro_ign]);
    
    //grau_avanco = matrix[procura_indice(100, vetor_map, 16)][procura_indice(rpm_anterior, vetor_rpm, 16)];
    //tempo_proxima_ignicao = tempo_atual + (tempo_cada_grau * grau_entre_cada_cilindro) + ((grau_pms * cilindro) * tempo_cada_grau);
    captura_dwell[1] = true;
    tempo_percorrido[1] = tempo_atual;
    digitalWrite(ignicao_pins[1],1);

  }

  if((tempo_atual - tempo_percorrido[1]) >= 4000){
    captura_dwell[1] = false;
    digitalWrite(ignicao_pins[1],0);
  }
 //IGN3
if ((captura_dwell[2] == false) && (tempo_proxima_ignicao[1] != 0) && (cilindro_ign == 3) && (tempo_atual - tempo_atual_proxima_ignicao[1] + (dwell_bobina * 1000) >= tempo_proxima_ignicao[1]) && (falha > 3) && (pms == 1) && (rpm >= 100))
{
  tempo_proxima_ignicao[1] = 0;
    cilindro++;
    cilindro_ign = 4;
  tempo_atual_proxima_ignicao[2] = tempo_atual_proxima_ignicao[1];
  //tempo_proxima_ignicao[0] = (grau_pms + grau_entre_cada_cilindro) * tempo_cada_grau;
  //tempo_proxima_ignicao[1] = (grau_pms + (grau_entre_cada_cilindro * 2)) * tempo_cada_grau;
  tempo_proxima_ignicao[2] = (grau_pms - grau_avanco + (grau_entre_cada_cilindro * 3)) * tempo_cada_grau;  

  //Serial.print(cilindro_ign);
  //Serial.print(" x ");
  //Serial.println(tempo_proxima_ignicao[cilindro_ign]);
    
    //grau_avanco = matrix[procura_indice(100, vetor_map, 16)][procura_indice(rpm_anterior, vetor_rpm, 16)];
    //tempo_proxima_ignicao = tempo_atual + (tempo_cada_grau * grau_entre_cada_cilindro) + ((grau_pms * cilindro) * tempo_cada_grau);
    captura_dwell[2] = true;
    tempo_percorrido[2] = tempo_atual;
    digitalWrite(ignicao_pins[2],1);

  }

  if((tempo_atual - tempo_percorrido[2]) >= 4000){
    captura_dwell[2] = false;
    digitalWrite(ignicao_pins[2],0);
  } 

  tempo_anterior = tempo_atual;
  interrupts();
}

void setup()
{

   // Chama a função para inicializar os valores da tabela de ponto grava e le caso nao use a interface 
  //inicializar_valores();
  //delay(1000);
  //gravar_dados_eeprom();
  //delay(1000);
  //aqui le os dados da eeprom que forem salvo anteriormente
  //ler_dados_eeprom();
  //delay(1000);

  pinMode(ign1, OUTPUT);
  pinMode(ign2, OUTPUT);
  pinMode(ign3, OUTPUT);
  pinMode(ign4, OUTPUT);
  pinMode(pino_sensor_roda_fonica, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pino_sensor_roda_fonica), leitor_sensor_roda_fonica, RISING);
  Serial.begin(9600);
}

void loop()
{    
  leitura_entrada_dados_serial();
    
//   // verifica se já passou o intervalo de tempo
  if (millis() - ultima_execucao >= intervalo_execucao)

  {    
   rpm_anterior = rpm;  
  // executa a função desejada
  // atualiza o tempo da última execução
   ultima_execucao = millis();

  }
}
