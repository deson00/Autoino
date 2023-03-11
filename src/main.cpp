#include <Arduino.h>

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
volatile unsigned long tempo_proxima_ignicao = 0;
volatile unsigned long tempo_atual = 0;
volatile unsigned long tempo_inicio_dwell;
volatile unsigned long tempo_final_dwell;
volatile unsigned long intervalo_tempo_entre_dente = 0;
volatile unsigned long verifica_falha = 0;
volatile int pms = 0;
volatile long falha = 0;
volatile int cilindro = 0;
volatile int cilindro_anterior = 0;
volatile int cilindro_ign = 0;
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
volatile bool captura_dwell = false;
volatile unsigned long tempo_percorrido;
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

void acionar_ignicao(int ign)
{
  if (ign > 0)
  {
  //int pino, int duracao
    digitalWrite(ignicao_pins[ign-1], HIGH);
    delayMicroseconds(10); // aguarda alguns microssegundos para garantir que o pino tenha mudado de estado
    unsigned long tempo_inicio_pulso = micros(); // armazena o tempo de início do pulso
    while (micros() - tempo_inicio_pulso < 4000) {
        // mantém o pino em nível alto por 4 ms
    }
    tempo_inicio_pulso = micros(); // armazena o tempo de início do pulso
    digitalWrite(ignicao_pins[ign-1], LOW);
  while (micros() - tempo_inicio_pulso < 4000) {
        // mantém o pino em nível alto por 4 ms
    }
    // Aguarda o tempo restante para completar a duração total do pulso
    // O valor 10 foi escolhido para compensar o tempo gasto nas instruções anteriores
    //delayMicroseconds((duracao * 1000) - 10 - 4000);
  }
  
}

// void acionar_pino(int pino, int duracao)
// {
//     digitalWrite(pino, HIGH); // aciona o pino
//     delayMicroseconds(10); // aguarda alguns microssegundos para garantir que o pino tenha mudado de estado
//     digitalWrite(pino, LOW); // desativa o pino

//     // Aguarda o tempo restante para completar a duração total do pulso
//     // O valor 10 foi escolhido para compensar o tempo gasto nas instruções anteriores
//     delayMicroseconds((duracao * 1000) - 10);
// }


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
    if (data == 'm')
    {
      tipo_vetor_map = 1;
    }
    if (data == 'r')
    {
      tipo_vetor_rpm = 1;
    }
    if (data == 'p')
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

int map_rpm_adiciona_ponto()
{
  int ponto_ignicao = matrix[procura_indice(96, vetor_map, 16)][procura_indice(rpm_anterior, vetor_rpm, 16)];
  if (ponto_ignicao <= matrix[15][15])
  {
    return ponto_ignicao;
  }
    return 1;
  
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
    tempo_proxima_ignicao = tempo_atual + (grau_pms * tempo_cada_grau) - (map_rpm_adiciona_ponto() * tempo_cada_grau);
    pms = 1;
    cilindro = 1;
  }
  posicao_atual_sensor = posicao_atual_sensor + grau_cada_dente;


if ((captura_dwell == false) && (tempo_atual + (dwell_bobina * 1000) >= tempo_proxima_ignicao) && (falha > 3) && (pms == 1) && (rpm >= 100))
{
    tempo_proxima_ignicao = tempo_atual + (tempo_cada_grau * grau_entre_cada_cilindro) - (map_rpm_adiciona_ponto() * tempo_cada_grau);
    cilindro_ign = cilindro;
    captura_dwell = true;
    tempo_percorrido = tempo_atual;
    digitalWrite(ignicao_pins[cilindro_ign-1],1);
    cilindro++;
    if(cilindro > 3){
      cilindro = 0;
    }
    }
Serial.println(map_rpm_adiciona_ponto());

  if((tempo_atual - tempo_percorrido) >= 4000){
    digitalWrite(ignicao_pins[cilindro_ign-1],0);
    captura_dwell = false;
  }

  tempo_anterior = tempo_atual;
  interrupts();
}

void setup()
{
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
    rpm_anterior = rpm;
    rpm = 0;
    
  // verifica se já passou o intervalo de tempo
  if (millis() - ultima_execucao >= intervalo_execucao)
  {
    // executa a função desejada
    // atualiza o tempo da última execução
    ultima_execucao = millis();
  }
}
